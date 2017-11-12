import numpy as np
import pandas as pd
import time
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
        rocks_df = pd.DataFrame({'distance':Rover.rock_dists, 'angles':Rover.rock_angles})
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
            if len(rocks_df) > 5 :
                Rover.mode = 'collection'
            if Rover.stopped_timestamp is None :
                Rover.stopped_timestamp = Rover.total_time
            elif (Rover.total_time - Rover.stopped_timestamp) > 10:
                Rover.tmp_ts = Rover.total_time
                Rover.mode = 'stuck'

            if (Rover.roll < 355 and Rover.roll >180) or (Rover.roll > 5 and Rover.roll < 180):
                Rover.mode = 'sandtrap'

            if Rover.samples_collected == Rover.samples_to_find :
                print('LOOKING FOR HOME...')
                distance_home = np.sqrt(np.sum([(Rover.pos[ix]-Rover.starting_pos[ix])**2 for ix in range(2)]))
                print(distance_home)
                if distance_home < 12 :
                    Rover.mode = 'home'
                else:
                    Rover.mode = Rover.mode




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

        elif Rover.mode == 'collection':
            if Rover.vel > 0.4:
                Rover.throttle = 0
                Rover.brake = 0.075
                Rover.steer = 0
                Rover.tmp_yaw = Rover.yaw
            else :
                if Rover.tmp_yaw is None :
                    Rover.tmp_yaw = Rover.yaw
                # Rover is close to or stopped
                if (Rover.total_time - Rover.stopped_timestamp) > 10:
                    if np.equal(np.round(Rover.yaw), np.round(Rover.tmp_yaw)):
                        Rover.mode = 'forward'
                if len(rocks_df) == 0 :
                    # rotate until rock is visible
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = 7
                else  :
                    # rock is visible
                    if np.abs(np.mean(rocks_df.angles)) < 0.12:
                        # rock is centered
                        if Rover.near_sample == 0:
                            # not close to rock yet
                            if Rover.vel < 0.35:
                                # inch forward...
                                if Rover.vel == 0:
                                    Rover.steer = -15
                                else :
                                    Rover.steer = 0
                                Rover.throttle = 0.2
                                Rover.brake = 0
                            else :
                                # but don't exceed 0.4 m/s
                                Rover.steer = 0
                                Rover.throttle = 0
                                Rover.brake = 0
                        else :
                            # rock is close enough to pick up
                            Rover.steer = 0
                            Rover.throttle = 0.0
                            Rover.brake = Rover.brake_set
                    else :
                        # rotate until the rock is centered
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = 7

        elif Rover.mode == 'stuck':
            print('STUCK FOR {} seconds'.format(
                Rover.total_time - Rover.tmp_ts))
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
            elif (Rover.total_time - Rover.tmp_ts) < 5.0:
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

        elif Rover.mode == 'home':
            print('HOME IS CLOSEBY!  Navigating to {}'.format(Rover.starting_pos))

            # home is within 12 m and all samples collected.
            if Rover.vel > 0.4:
                # if moving, stop
                Rover.throttle = 0
                Rover.brake = 0.2
                Rover.steer = 0
            else :
                #if stopped, look for starting_pos
                if ~np.all(np.equal(np.round(Rover.starting_pos), np.round(Rover.pos))):
                    # we're not home yet
                    #calculate difference in angle between yaw and starting_pos
                    d_dest = np.array(Rover.starting_pos) - np.array(Rover.pos)

                    rov_ux = np.cos(np.deg2rad(Rover.yaw))
                    rov_uy = np.sin(np.deg2rad(Rover.yaw))

                    phi = np.arccos(
                            (rov_ux*d_dest[0] + rov_uy*d_dest[1]) /
                            (1*np.sqrt((d_dest[0]**2) + (d_dest[1]**2))))
                    print('PHI: {}'.format(phi))
                    if np.abs(phi) >= 0.2 :
                        # if not pointing at starting_pos, rotate
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = np.clip(np.rad2deg(phi), -10, 10)
                    else :
                        # pointing at the starting pos, inch forward
                        if Rover.vel < 0.3 :
                            # dont go too fast
                            Rover.throttle = 0.2
                            Rover.brake = 0
                            Rover.steer = np.clip(np.rad2deg(phi), -10, 10)
                        else :
                            # coasting
                            Rover.throttle = 0.0
                            Rover.brake = 0
                            Rover.steer = np.clip(np.rad2deg(phi), -10, 10)
                else :
                    # stop upon arrival
                    Rover.brake = Rover.brake_set
                    Rover.throttle = 0
                    Rover.steer = 0
                    print('MADE IT HOME.  MISSION COMPLETE!')




    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
        Rover.mode = 'forward'

    return Rover
