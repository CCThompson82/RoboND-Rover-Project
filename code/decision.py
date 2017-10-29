import numpy as np
import pandas as pd

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        print('MODE: {}'.format(Rover.mode))

        obs_df = pd.DataFrame({'distance':Rover.obs_dists, 'angles':Rover.obs_angles})
        # right = -35
        # left = 10
        # max_threshold = 90 #NOTE: Lower equals more wall hugging
        # yaws = []
        # distances = []
        # yaws_df = pd.DataFrame([], columns = ['normal', 'tangent','distance','yaw',], index=[])
        # headings = range(right,left, 2)
        # for phi in headings:
        #     phi = np.deg2rad(phi)
        #
        #     flt = np.random.random() / 1e4
        #     phi_set =obs_df[(obs_df.angles > phi-0.05) & (obs_df.angles < phi+0.05)]
        #     if len(phi_set) == 0 :
        #         ar = 0
        #         br = 0
        #     else:
        #         a = np.min(phi_set.angles)
        #         dists = [np.abs(a - angle) for angle in phi_set.angles]
        #
        #         ar = phi_set.distance[np.argmin(dists)]
        #         b = np.max(phi_set.angles)
        #         dists = [np.abs(b - angle) for angle in phi_set.angles]
        #         br = phi_set.distance[np.argmin(dists)]
        #
        #
        #     if (ar > max_threshold) & (br > max_threshold) :
        #         normal = 0
        #         tangent = normal - (np.pi/2)
        #     elif (ar == br) & (br == 0) :
        #         normal = np.pi
        #         tangent = np.pi - (np.pi/2)
        #     else :
        #         ax, ay = np.sin(a)*ar, np.cos(a)*ar
        #         bx, by = np.sin(b)*br, np.cos(b)*br
        #
        #         normal = ((bx - ax) / (ay - by)) + np.pi/2
        #         tangent = normal - (np.pi/2)
        #     yaw = normal - (np.pi/2)
        #     yaws.append(yaw)
        #     distances.append(np.mean([ar,br]))
        #     yaws_df = yaws_df.append(pd.DataFrame({'normal':normal, 'tangent':tangent, 'yaw':yaw,
        #                      'distance':np.min([ar,br])}, index = [phi]))
        #
        # if np.sum(yaws_df.distance < 10) > 0 :
        #     m2 = np.median(yaws_df.yaw)
        # else :
        #     m1 = np.mean(Rover.nav_angles)
        #     weights1 = np.clip(max_threshold - np.array(yaws_df.distance), 20, max_threshold)
        #     weights1[yaws_df.normal == 0] = 0.1
        #     weights1 = weights1/np.linalg.norm(weights1)
        #     weights2 = np.abs(1 - yaws_df.index.astype(np.float32))
        #     weights2 = weights2/np.linalg.norm(weights2)
        #     try :
        #         weights = weights1 * weights2
        #     except:
        #         print(weights1, weights2)
        #     weights = weights / np.linalg.norm(weights)
        #     m2 = np.average(yaws_df.yaw, weights = weights)
        #     yaws_df['weights']=weights
        # print(yaws_df)
        #
        # m4 = m2
        if Rover.stopped_timestamp is None :
            Rover.stopped_timestamp = Rover.total_time
        elif (Rover.total_time - Rover.stopped_timestamp) > 10:
            Rover.mode = 'stuck'


        COMPASS_YAW = np.deg2rad(-35)
        STIFFARM = 15
        phi_set = obs_df[(obs_df.angles > COMPASS_YAW-0.05) & (obs_df.angles < COMPASS_YAW+0.05)]
        if len(phi_set) == 0 :
            compass_dist = 0
        else :

            compass_dist = np.min(phi_set.distance)
        m4 = np.deg2rad(STIFFARM - compass_dist)
        print('Distance to wall: {}'.format(compass_dist))

        phi_set = obs_df[(obs_df.angles > -0.05) & (obs_df.angles < 0.05)]
        distance_ahead = np.min(phi_set.distance)
        print('DISTANCE AHEAD: {}'.format(distance_ahead))

        # Check the extent of navigable terrain

        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # TODO: logic for when to slow down
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            Rover.brake = 0
            # Set steering to average angle clipped to the range +/- 15
            print('AVERAGE NAV YAW: {}'.format(np.median(Rover.nav_angles)))
            print('TARGET YAW: {}'.format(m4))
            Rover.steer = np.clip(m4 * 180/np.pi, -15, 15)
            print('ROVER STEERS: {}'.format(Rover.steer))

            if distance_ahead < Rover.stop_forward and compass_dist < Rover.stop_forward:
                Rover.mode = 'stop'


        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if distance_ahead < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if distance_ahead >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(m4, -15, 15)
                    Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            print('ROVER IS STUCK FIX ME')


    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
