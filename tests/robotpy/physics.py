from pyfrc.physics import drivetrains
from pyfrc.physics.units import units

import math

class PhysicsEngine(object):
    def __init__(self, physics_controller):
        """
            :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        """

        self.physics_controller = physics_controller

    def update_sim(self, hal_data, now, tm_diff):
        """
            Called when the simulation parameters for the program need to be
            updated.

            :param now: The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
        """

        # Simulate the drivetrain
        can = hal_data["CAN"]
        lr_drive = can[11]["value"]
        rr_drive = can[21]["value"]
        lf_drive = can[31]["value"]
        rf_drive = can[41]["value"]

        # Scale the steering motor to simulate gearing
        # Full throttle gives us about 1rev/s

        for idx in [12, 22, 32, 42]:
            speed = math.ceil(4096 * 1 * can[idx]["value"] * tm_diff)
            #print("speed: %f" % speed)
            #print("motor: %f" % can[idx]["value"])
            can[idx]["quad_position"] += speed
            can[idx]["quad_velocity"] = speed

        lr_angle = -can[12]["quad_position"] / 4096.0 * 360.0
        rr_angle = -can[22]["quad_position"] / 4096.0 * 360.0
        lf_angle = -can[32]["quad_position"] / 4096.0 * 360.0
        rf_angle = -can[42]["quad_position"] / 4096.0 * 360.0

        # Dimensions are all in feet
        m_to_ft = 3.28
        # The drivetrain model has weird axes. Remap them, and flip afterwards.
        vy, vx, vw = drivetrains.four_motor_swerve_drivetrain(
            lr_drive, rr_drive, lf_drive, rf_drive,
            lr_angle, rr_angle, lf_angle, rf_angle,
            x_wheelbase = 1.0 * m_to_ft, y_wheelbase = 1.0 * m_to_ft,
            speed = 3.0 * m_to_ft
        )
        vy = -vy
        vw = -vw
        self.physics_controller.vector_drive(-vy, vx, -vw, tm_diff)
