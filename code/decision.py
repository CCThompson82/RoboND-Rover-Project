import numpy as np



class Explore(object):

    def __init__(self):

        self.counter = 0
        self.init_yaw = None
        self.max_dist_heading = None
        self.max_dist = None

class WallCrawler(object):

    def __init__(self, set_heading ):

        self.init_heading = set_heading


def FindHeading(object):

    def __init__(self, target_heading):

        self.target_heading = target_heading

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # TODO: Refactor to update necessary attr based on the
    # mode selected.
    if Rover.mode == 'explore':

        if Rover.explore_sm.counter == 0:
            # init the explore state machine
            Rover.explore_sm.init_yaw = Rover.yaw
            Rover.explore_sm.target_yaw = Rover.yaw + 10
            Rover.explore_sm.max_dist_heading = Rover.yaw
            Rover.explore_sm.max_dist = 0
            Rover.explore_sm.counter += 1

        elif (np.round(Rover.yaw) == np.round(Rover.explore_sm.target_yaw) and
                Rover.explore_sm.counter > 100):
            print('Switching to `find_heading` mode')
            Rover.mode = 'find_heading'
            Rover.throttle = 0
            Rover.steer = 0
            Rover.brake = 10
            print(Rover.explore_sm.max_dist_heading)
            Rover.find_heading.target_heading = Rover.explore_sm.max_dist_heading
            print(Rover.find_heading)
            Rover.explore_sm = Explore()
        else:
            print('`explore` mode activated')
            # spin and explore
            # what is the max distance rover can observe straight ahead
            filtered_angles = [Rover.yaw - 1 < x < Rover.yaw + 1 for x in Rover.nav_angles]
            if len(Rover.nav_dists[filtered_angles]) < 10 :
                # nothing to remember here
                pass
            else :
                if np.max(Rover.nav_dists) > Rover.explore_sm.max_dist :
                    Rover.explore_sm.max_dist = np.max(Rover.nav_dists)
                    Rover.explore_sm.max_dist_heading = Rover.yaw
                    print(Rover.explore_sm)
                else:
                    pass
            Rover.throttle = 0
            Rover.steer = 5
        Rover.explore_sm.counter += 1

    elif Rover.mode == 'find_heading':
        Rover.throttle = 0
        Rover.brake = 0
        if np.round(Rover.yaw) == np.round(Rover.find_heading.target_heading):
            Rover.mode = 'wall_crawler'
            Rover.find_heading = None
            Rover.brake = 10
            Rover.steer = 0
        elif Rover.yaw - Rover.find_heading.target_yaw < 0 :
            # turn left
            Rover.steer = -5
        else  :
            Rover.steer = 5

    elif Rover.mode == 'wall_crawler':
        print('Waiting for wall_crawler instructions!')



    elif Rover.mode == 'pick_and_place':
        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
        distance_to_rock = np.mean(Rover.rock_dists)

        Rover.brake = 0
        if distance_to_rock > 10 :
            Rover.throttle = 0.5
        elif 2 < distance_to_rock < 10:
            Rover.throttle = 0.1
        elif 1 < distance_to_rock < 2 :
            Rover.throttle = 0.01
        elif distance_to_rock < 1 :
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
    #
    #
    #
    #
    #
    #
    #
    #


    #
    # # Example:
    # # Check if we have vision data to make decisions with
    # if Rover.nav_angles is not None:
    #     # Check for Rover.mode status
    #     if Rover.mode == 'forward':
    #         # Check the extent of navigable terrain
    #         if len(Rover.nav_angles) >= Rover.stop_forward:
    #             # If mode is forward, navigable terrain looks good
    #             # and velocity is below max, then throttle
    #             if Rover.vel < Rover.max_vel:
    #                 # Set throttle value to throttle setting
    #                 Rover.throttle = Rover.throttle_set
    #             else: # Else coast
    #                 Rover.throttle = 0
    #             Rover.brake = 0
    #             # Set steering to average angle clipped to the range +/- 15
    #             Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    #         # If there's a lack of navigable terrain pixels then go to 'stop' mode
    #         elif len(Rover.nav_angles) < Rover.stop_forward:
    #                 # Set mode to "stop" and hit the brakes!
    #                 Rover.throttle = 0
    #                 # Set brake to stored brake value
    #                 Rover.brake = Rover.brake_set
    #                 Rover.steer = 0
    #                 Rover.mode = 'stop'
    #
    #     # If we're already in "stop" mode then make different decisions
    #     elif Rover.mode == 'stop':
    #         # If we're in stop mode but still moving keep braking
    #         if Rover.vel > 0.2:
    #             Rover.throttle = 0
    #             Rover.brake = Rover.brake_set
    #             Rover.steer = 0
    #         # If we're not moving (vel < 0.2) then do something else
    #         elif Rover.vel <= 0.2:
    #             # Now we're stopped and we have vision data to see if there's a path forward
    #             if len(Rover.nav_angles) < Rover.go_forward:
    #                 Rover.throttle = 0
    #                 # Release the brake to allow turning
    #                 Rover.brake = 0
    #                 # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    #                 Rover.steer = -15 # Could be more clever here about which way to turn
    #             # If we're stopped but see sufficient navigable terrain in front then go!
    #             if len(Rover.nav_angles) >= Rover.go_forward:
    #                 # Set throttle back to stored value
    #                 Rover.throttle = Rover.throttle_set
    #                 # Release the brake
    #                 Rover.brake = 0
    #                 # Set steer to mean angle
    #                 Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    #                 Rover.mode = 'forward'
    # # Just to make the rover do something
    # # even if no modifications have been made to the code
    # else:
    #     Rover.throttle = -Rover.throttle_set
    #     Rover.steer = 0
    #     Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
