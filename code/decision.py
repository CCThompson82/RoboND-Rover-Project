import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # TODO: Update attr necessary to determine mode
    Rover.historical_vel.append(Rover.vel)
    Rover.historical_pos.append(Rover.pos)

    #######################################################
    if len(Rover.historical_velocity) > 20:
        if np.mean(Rover.historical_velocity[-20:]) < 0.05 :
            if len(Rover.nav_angles) >= Rover.go_forward:
                Rover.mode = 'explore'
            else:
                Rover.mode = 'stuck'
    elif len(Rover.rock_angles) > 0 :
        Rover.mode = 'pick_and_place'
    elif len(Rover.nav_angles) == 0 :
        Rover.mode = 'stuck'
    elif len(Rover.nav_angles) > 0 :
        Rover.mode = 'explore'
    elif Rover.samples_collected == 6 :
        Rover.mode = 'go_home'
    else:
        Rover.mode = 'stuck'

    # TODO: Refactor to update necessary attr based on the
    # mode selected.

    if Rover.mode == 'stuck':
        Rover.throttle = -Rover.throttle_set
        Rover.steer = (np.random.random() * 30) - 15
        Rover.brake = 0
    elif Rover.mode == 'explore':
        # choose a yaw
        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
        steer_deg = Rover.steer * np.pi / 180
        # how far can rover see on that heading
        horizon_dist = np.max(
            Rover.dist[
                (steer_deg - 2) < Rover.nav_angles <
                (steer_deg + 2)])
        target_vel = -0.2 + ( 1.2 / (1 + exp(-0.05*(
                horizon_dist-100))))

        vel_diff = Rover.vel - (target_vel*Rover.max_vel)
        Rover.throttle = Rover.throttle_set * (vel_diff/Rover.max_vel)

        if Rover.throttle < 0 :
            Rover.brake = Rover.brake_set * np.abs(vel_diff/Rover.max_vel)
        else:
            Rover.brake = 0

    elif Rover.mode == 'pick_and_place':
        Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
        distance_to_rock = np.mean(Rover.rock_dists)

        Rover.brake = 0
        if distance_to_rock > 10 :
            Rover.throttle = 0.5
        elif 2 < distance_to_rock < 10:
            Rover.throttle = 0.1
        elif 1 < distance_to_rock < 2 :
            Rover.throttle = 0.01 :
        elif distance_to_rock < 1 :
            Rover.throttle = 0
            Rover.brake = Rover.brake_set 










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
