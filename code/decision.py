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
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            # if len(Rover.nav_angles) >= Rover.stop_forward:
            #     # If mode is forward, navigable terrain looks good
            #     # and velocity is below max, then throttle

            # TODO: logic for when to slow down
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            Rover.brake = 0
            # TODO: Add rover.steer logic
            obs_df = pd.DataFrame({'distance':Rover.obs_dists, 'angles':Rover.obs_angles})
            yaws = []
            distances = []
            headings = range(-45,50,5)
            for phi in headings:
                phi = np.deg2rad(phi)
                ar = np.min(obs_df[(obs_df.angles > phi-0.01) & (obs_df.angles < phi+0.01)].distance)
                a = np.min(obs_df[(obs_df.angles > phi-0.01) & (obs_df.angles < phi+0.01)].angles)
                br = np.max(obs_df[(obs_df.angles > phi-0.01) & (obs_df.angles < phi+0.01)].distance)
                b = np.max(obs_df[(obs_df.angles > phi-0.01) & (obs_df.angles < phi+0.01)].angles)

                max_threshold = 100
                if (ar > max_threshold) & (br > max_threshold) :
                    normal = 0
                else :
                    ax, ay = np.sin(a)*ar, np.cos(a)*ar
                    bx, by = np.sin(b)*br, np.cos(b)*br

                    normal = ((bx - ax) / (ay - by))
            #     print('tangent angle: {}'.format(theta))
                yaw = normal #+ (np.pi/2)
                # yaw = np.pi/2
                yaws.append(yaw)
                distances.append(np.mean([ar,br]))

            m1 = np.mean(yaws)
            m2 = np.average(yaws, weights=[1/d for d in distances])
            m3 = np.average(yaws, weights = [(phi - np.abs(h)) for h in headings]/(phi))
            m4 = np.mean([m3,m2])
            # Set steering to average angle clipped to the range +/- 15
            Rover.steer = np.clip(m4 * 180/np.pi, -15, 15)
            print('ROVER STEERS: {}'.format(Rover.steer))
            print('TARGET YAW: {}'.format(m3))

            # # If there's a lack of navigable terrain pixels then go to 'stop' mode
            # elif len(Rover.nav_angles) < Rover.stop_forward:
            #         # Set mode to "stop" and hit the brakes!
            #         Rover.throttle = 0
            #         # Set brake to stored brake value
            #         Rover.brake = Rover.brake_set
            #         Rover.steer = 0
            #         Rover.mode = 'stop'

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
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -25, 25)
                    Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = -Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
