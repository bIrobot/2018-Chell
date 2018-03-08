#
# See the notes for the other physics sample
#


from pyfrc.physics import drivetrains


class PhysicsEngine(object):
    '''
       Simulates a 4-wheel robot using Tank Drive joystick control
    '''

    def __init__(self, physics_controller):
        '''
            :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        '''

        self.physics_controller = physics_controller

    def update_sim(self, hal_data, now, tm_diff):
        '''
            Called when the simulation parameters for the program need to be
            updated.

            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        '''

        # Simulate the drivetrain
        rr_motor = hal_data['CAN'][1]['value']
        lr_motor = hal_data['CAN'][2]['value']
        rf_motor = hal_data['CAN'][3]['value']
        lf_motor = hal_data['CAN'][4]['value']

        speed, rotation = drivetrains.four_motor_drivetrain(lr_motor * -1, rr_motor * -1, lf_motor * -1, rf_motor * -1, x_wheelbase=2, speed=6, deadzone=drivetrains.linear_deadzone(0.1))
        self.physics_controller.drive(speed, rotation, tm_diff)