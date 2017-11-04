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
        nav_df = pd.DataFrame({'distance':Rover.nav_dists, 'angles':Rover.nav_angles})
        # COMPASS_YAW = np.deg2rad(-30)
        STIFFARM = 12
        # phi_set = obs_df[(obs_df.angles > COMPASS_YAW-0.05) & (obs_df.angles < COMPASS_YAW+0.05)]
        # if len(phi_set) == 0 :
        #     compass_dist = 0
        # else :
        #     compass_dist = np.min(phi_set.distance)
        # m4 = np.deg2rad(STIFFARM - compass_dist)
        # print('Distance to wall: {}'.format(compass_dist))
        #
        # phi_set = obs_df[(obs_df.angles > -0.05) & (obs_df.angles < 0.05)]
        # distance_ahead = np.min(phi_set.distance)
        # print('DISTANCE AHEAD: {}'.format(distance_ahead))

        wall_diffs = []
        for rad in range(-35,-20,5):
            phi = np.deg2rad(rad)
            phi_set = obs_df[(obs_df.angles > phi-0.05) & (obs_df.angles < phi+0.05)]
            if len(phi_set) == 0 :
                wall_dist = 0
            else :
                wall_dist = np.min(phi_set.distance)
            wall_diffs.append(np.abs(STIFFARM/np.sin(phi))-wall_dist)

        m4 = np.deg2rad(np.mean(wall_diffs))
        print(m4)
        # #TODO: Switch to nav terrain not obstacles
        nav_set = nav_df[(nav_df.angles > -0.1) & (nav_df.angles < 0.1)]
        obs_set = obs_df[(obs_df.angles > -0.1) & (obs_df.angles < 0.1)]
        # print(min(nav_set.distance), max(nav_set.distance), min(obs_set.distance), max(obs_set.distance))
        # if len(phi_set) == 0:
        #     distance_ahead = 0
        # elif len(phi_set) < 10:
        #     distance_ahead = 0
        # else :
        #     distance_ahead = np.percentile(phi_set.distance,10)
        # print('DISTANCE AHEAD: {}'.format(distance_ahead))


        # Check the extent of navigable terrain

        # Check for Rover.mode status
        if Rover.mode == 'forward':
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


            if len(nav_set) < Rover.stop_forward :#and compass_dist < Rover.stop_forward:
                Rover.mode = 'stop'

            if Rover.stopped_timestamp is None :
                Rover.stopped_timestamp = Rover.total_time
            elif (Rover.total_time - Rover.stopped_timestamp) > 10:
                Rover.tmp_ts = Rover.total_time
                Rover.mode = 'stuck'

            if (Rover.roll < 355 and Rover.roll >180) or (Rover.roll > 5 and Rover.roll < 180):
                Rover.mode = 'sandtrap'



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
                if len(nav_set)< Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(nav_set) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(m4, -15, 15)
                    Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            print('STUCK FOR {} seconds'.format(Rover.total_time - Rover.tmp_ts))
            if (Rover.total_time - Rover.tmp_ts) < 1.5:
                Rover.steer = -15
                Rover.brake = 0
                Rover.throttle = -0.5
                Rover.mode = 'stuck'
            elif (Rover.total_time - Rover.tmp_ts) < 3.0:
                Rover.steer = 15
                Rover.brake = 0
                Rover.throttle = 0
                Rover.mode = 'stuck'
            elif (Rover.total_time - Rover.tmp_ts) < 4.5:
                Rover.steer = 15
                Rover.brake = 0
                Rover.throttle = 0.5
                Rover.mode = 'stuck'
            elif (Rover.total_time - Rover.tmp_ts) > 6.0:
                Rover.stopped_timestamp = Rover.total_time
                Rover.tmp_ts = None
                Rover.mode = 'forward'
        elif Rover.mode == 'sandtrap':
            Rover.steer = 15
            Rover.throttle = 0
            Rover.brake = 0
            if (Rover.roll > 355.0) or (Rover.roll < 5.0):
                Rover.mode = 'forward'


    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover
