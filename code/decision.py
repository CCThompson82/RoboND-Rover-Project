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
        # organize sensor information lists
        obs_df = pd.DataFrame({'distance':Rover.obs_dists, 'angles':Rover.obs_angles})
        nav_df = pd.DataFrame({'distance':Rover.nav_dists, 'angles':Rover.nav_angles})
        rocks_df = pd.DataFrame({'distance':Rover.rock_dists, 'angles':Rover.rock_angles})
        STIFFARM = 7
        wall_diffs = []
        for rad in range(-35,-20,5):
            phi = np.deg2rad(rad)
            phi_set = obs_df[(obs_df.angles > phi-0.05) & (obs_df.angles < phi+0.05)]
            if len(phi_set) == 0:
                wall_dist = 0
            else:
                wall_dist = np.min(phi_set.distance)
            wall_diffs.append(np.abs(STIFFARM/np.sin(phi))-wall_dist)

        steer_rad = np.deg2rad(np.mean(wall_diffs))
        nav_set = nav_df[(nav_df.angles > -0.1) & (nav_df.angles < 0.1)]
        go_set = nav_df[(nav_df.angles > -0.4) & (nav_df.angles < -0.2)]

        # Check for Rover.mode status
        if Rover.mode == 'forward':
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = np.clip(steer_rad * 180/np.pi, -15, 15)

            # change mode under various circumstances
            if len(nav_set) < Rover.stop_forward:
                Rover.mode = 'stop'
            if len(rocks_df) > 1:
                Rover.mode = 'collection'
            if Rover.stopped_timestamp is None:
                Rover.stopped_timestamp = Rover.total_time
            elif (Rover.total_time - Rover.stopped_timestamp) > 7:
                Rover.tmp_ts = Rover.total_time
                Rover.mode = 'stuck'

            if (Rover.roll < 355 and Rover.roll > 180) or (Rover.roll > 5 and Rover.roll < 180):
                Rover.mode = 'sandtrap'

            if Rover.samples_collected == Rover.samples_to_find:
                print('LOOKING FOR HOME...')
                distance_home = np.sqrt(np.sum(
                    [(Rover.pos[ix]-Rover.starting_pos[ix])**2 for
                     ix in range(2)]))
                if distance_home < 12:
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
                if len(go_set) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line
                    # will induce 4-wheel turning
                    Rover.steer = 15
                # If we're stopped but see sufficient navigable terrain, go!
                if len(go_set) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(steer_rad * 180/np.pi, -15, 15)
                    Rover.mode = 'forward'

        elif Rover.mode == 'collection':
            if Rover.vel > 0.4:
                Rover.throttle = 0
                Rover.brake = 0.075
                Rover.steer = 0
                Rover.tmp_yaw = Rover.yaw
            else:
                if Rover.tmp_yaw is None:
                    Rover.tmp_yaw = Rover.yaw
                # Rover is close to or stopped
                if (Rover.total_time - Rover.stopped_timestamp) > 10:
                    if np.equal(np.round(Rover.yaw), np.round(Rover.tmp_yaw)):
                        Rover.mode = 'forward'
                    elif (Rover.total_time - Rover.stopped_timestamp) > 30:
                        Rover.mode = 'stuck'
                if len(rocks_df) == 0:
                    # rotate until rock is visible
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = 7
                else:
                    # rock is visible
                    if np.abs(np.mean(rocks_df.angles)) < 0.12:
                        # rock is centered
                        if Rover.near_sample == 0:
                            # not close to rock yet
                            if Rover.vel < 0.35:
                                # inch forward...
                                if Rover.vel == 0:
                                    Rover.steer = -15
                                else:
                                    Rover.steer = 0
                                Rover.throttle = 0.2
                                Rover.brake = 0
                            else:
                                # but don't exceed 0.4 m/s
                                Rover.steer = 0
                                Rover.throttle = 0
                                Rover.brake = 0
                        else:
                            # rock is close enough to pick up
                            Rover.steer = 0
                            Rover.throttle = 0.0
                            Rover.brake = Rover.brake_set
                    else:
                        # rotate until the rock is centered
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = 7

        elif Rover.mode == 'stuck':
            if Rover.tmp_ts is None:
                Rover.tmp_ts = Rover.total_time
            print('STUCK FOR {} seconds'.format(
                Rover.total_time - Rover.tmp_ts))
            if (Rover.total_time - Rover.tmp_ts) < 1.5:
                Rover.steer = -15
                Rover.brake = 0
                Rover.throttle = -0.5
                Rover.mode = 'stuck'
            elif (Rover.total_time - Rover.tmp_ts) < 3.5:
                Rover.steer = 15
                Rover.brake = 0
                Rover.throttle = 0
                Rover.mode = 'stuck'
            elif (Rover.total_time - Rover.tmp_ts) < 5.0:
                Rover.steer = 10
                Rover.brake = 0
                Rover.throttle = 0.5
                Rover.mode = 'stuck'
            elif (Rover.total_time - Rover.tmp_ts) > 7.0:
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
            else:
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
                    if phi >= np.pi :
                        phi = np.pi - phi
                    if np.abs(phi) >= 0.2:
                        # if not pointing at starting_pos, rotate
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = np.clip(np.rad2deg(phi), -10, 10)
                    else:
                        # pointing at the starting pos, inch forward
                        if Rover.vel < 0.3:
                            # dont go too fast
                            Rover.throttle = 0.2
                            Rover.brake = 0
                            Rover.steer = np.clip(np.rad2deg(phi), -10, 10)
                        else:
                            # coasting
                            Rover.throttle = 0.0
                            Rover.brake = 0
                            Rover.steer = np.clip(np.rad2deg(phi), -10, 10)
                else:
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
