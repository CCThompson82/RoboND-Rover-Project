import os
import numpy as np
import pandas as pd
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def find_rocks(img, rgb_thresh=(100, 100, 50)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the
    # center bottom of the image.
    x_pixel = -(ypos - binary_img.shape[0]).astype(float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle)
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):

    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image

    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO:
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 10
    y_offset =  6 + dst_size
    img = Rover.img
    src2dest = {'top_left':
                    {'src': np.float32([120,95]),
                     'dst': np.float32([(img.shape[1]/2) - (dst_size/2),
                                        img.shape[0]- y_offset - dst_size ])},
                'top_right':
                    {'src': np.float32([200,95]),
                     'dst': np.float32([(img.shape[1]/2) + (dst_size/2),
                                        img.shape[0]- y_offset - dst_size ])},
                'bottom_right':
                    {'src': np.float32([303,142]),
                     'dst': np.float32([(img.shape[1]/2) + (dst_size/2),
                                        img.shape[0]- y_offset + dst_size ])},
                'bottom_left':
                    {'src': np.float32([14, 142]),
                     'dst': np.float32([(img.shape[1]/2) - (dst_size/2),
                                        img.shape[0] - y_offset + dst_size ])}}
    # 2) Apply perspective transform
    keys = src2dest.keys()
    warped_img = perspect_transform(
                    Rover.img,
                    src = np.array([src2dest[k]['src'] for k in keys]),
                    dst = np.array([src2dest[k]['dst'] for k in keys]))
    mask = perspect_transform(np.ones_like(Rover.img[:,:,0]),
                    src = np.array([src2dest[k]['src'] for k in keys]),
                    dst = np.array([src2dest[k]['dst'] for k in keys]))
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain_binary = color_thresh(warped_img)
    obstacle_binary = (terrain_binary == 0)*mask
    rocks_binary = find_rocks(warped_img)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle_binary.astype(np.uint8) * 255
    Rover.vision_image[:,:,2] = terrain_binary.astype(np.uint8) * 255
    Rover.vision_image[:,:,1] = rocks_binary.astype(np.uint8) * 255

    # 5) Convert map image pixel values to rover-centric coords
    nogo_x, nogo_y = rover_coords(obstacle_binary)
    go_x, go_y = rover_coords(terrain_binary)
    rock_x, rock_y = rover_coords(rocks_binary)

    # 6) Convert rover-centric pixel values to world coordinates
    nogo_x_world, nogo_y_world = pix_to_world(nogo_x, nogo_y, xpos=Rover.pos[0], ypos=Rover.pos[1], yaw=Rover.yaw, world_size=200, scale=10)
    go_x_world, go_y_world = pix_to_world(go_x, go_y, xpos=Rover.pos[0], ypos=Rover.pos[1], yaw=Rover.yaw, world_size=200, scale=10)
    rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, xpos=Rover.pos[0], ypos=Rover.pos[1], yaw=Rover.yaw, world_size=200, scale=10)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[nogo_y_world, nogo_x_world, 0] += 1
    Rover.worldmap[go_y_world, go_x_world, 2] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1

    # Rover.worldmap[nogo_y_world, nogo_x_world, 2] -= 1
    Rover.worldmap[go_y_world, go_x_world, 0] -= 1
    Rover.worldmap = np.clip(Rover.worldmap, 0, 255)


    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    dist, angles = to_polar_coords(go_x, go_y)
    obs_dists, obs_angles = to_polar_coords(nogo_x, nogo_y)
    rock_dists, rock_angles = to_polar_coords(rock_x, rock_y)

    Rover.nav_dists = dist
    Rover.nav_angles = angles
    Rover.rock_dists = rock_dists
    Rover.rock_angles = rock_angles
    Rover.obs_dists = obs_dists
    Rover.obs_angles = obs_angles



    ############################################################################
    # Save data for analysis and exploration
    # try :
    #     _ = os.listdir('../.tmp')
    # except :
    #     os.makdirs('../.tmp')
    #
    # nav_df = pd.DataFrame({'distance': dist, 'angles': angles})
    # nav_df.to_csv('../.tmp/navigation_df.csv')
    # rocks_df = pd.DataFrame({'distance': rock_dists, 'angles': rock_angles})
    # rocks_df.to_csv('../.tmp/rocks_df.csv')
    # obs_df = pd.DataFrame({'distance': obs_dists, 'angles': obs_angles})
    # obs_df.to_csv('../.tmp/obs_df.csv')
    # pd.DataFrame(Rover.worldmap[:,:,0]).to_csv('../.tmp/obs_map.csv')
    # pd.DataFrame({'xpix':nogo_x, 'ypix':nogo_y}).to_csv('../.tmp/obs_pix.csv')
    ############################################################################
    return Rover
