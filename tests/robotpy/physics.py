from pyfrc.physics import drivetrains
from pyfrc.physics.units import units

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
        for idx in [12, 22, 32, 42]:
            speed = int(4096 * 4 * can[idx]["value"] * tm_diff)
            can[idx]["quad_position"] += speed
            can[idx]["quad_velocity"] = speed

        lr_angle = -can[12]["quad_position"] / 4096.0 * 360.0
        rr_angle = -can[22]["quad_position"] / 4096.0 * 360.0
        lf_angle = -can[32]["quad_position"] / 4096.0 * 360.0
        rf_angle = -can[42]["quad_position"] / 4096.0 * 360.0

        # Dimensions are all in feet
        m_to_ft = 3.28
        vx, vy, vw = drivetrains.four_motor_swerve_drivetrain(
            lr_drive, rr_drive, lf_drive, rf_drive,
            lr_angle, rr_angle, lf_angle, rf_angle,
            x_wheelbase = 1.0 * m_to_ft, y_wheelbase = 1.0 * m_to_ft,
            speed = 3.0 * m_to_ft
        )
        self.physics_controller.vector_drive(vx, vy, vw, tm_diff)
