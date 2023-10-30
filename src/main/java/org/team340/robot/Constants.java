package org.team340.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.SPI;
import org.team340.lib.drivers.ADIS16470.CalibrationTime;
import org.team340.lib.drivers.ADIS16470.IMUAxis;
import org.team340.lib.swerve.SwerveBase.SwerveMotorType;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double PERIOD = 0.020;
    public static final double VOLTAGE = 12.0;
    public static final double FIELD_LENGTH = 16.5417;
    public static final double FIELD_WIDTH = 8.0136;

    /**
     * Driver and co-driver controller constants.
     */
    public static final class ControllerConstants {

        public static final int DRIVER = 0;
        public static final int CO_DRIVER = 1;

        public static final double JOYSTICK_DEADBAND = 0.1;
        public static final double JOYSTICK_THRESHOLD = 0.7;
        public static final double TRIGGER_DEADBAND = 0.1;
        public static final double TRIGGER_THRESHOLD = 0.1;

        public static final double DRIVE_EXP = 1.0;
        public static final double DRIVE_MULTIPLIER = 0.75;
        public static final double DRIVE_MULTIPLIER_MODIFIED = 0.95;

        public static final double DRIVE_ROT_EXP = 2.0;
        public static final double DRIVE_ROT_MULTIPLIER = 0.4;
    }

    /**
     * Map of hardware device IDs.
     */
    public static final class RobotMap {

        public static final class CANBus {

            public static final int FRONT_LEFT_MOVE = 2;
            public static final int FRONT_LEFT_TURN = 3;
            public static final int BACK_LEFT_MOVE = 4;
            public static final int BACK_LEFT_TURN = 5;
            public static final int BACK_RIGHT_MOVE = 6;
            public static final int BACK_RIGHT_TURN = 7;
            public static final int FRONT_RIGHT_MOVE = 8;
            public static final int FRONT_RIGHT_TURN = 9;
        }
    }

    /**
     * Constants for the swerve subsystem.
     */
    public static final class SwerveConstants {

        private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
            .setLabel("Front Left")
            .useSparkMaxAttachedEncoder(0.0, false)
            .setPosition(0.3000, 0.3000)
            .setMoveMotor(RobotMap.CANBus.FRONT_LEFT_MOVE, true, true)
            .setTurnMotor(RobotMap.CANBus.FRONT_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
            .setLabel("Back Left")
            .useSparkMaxAttachedEncoder(0.0, false)
            .setPosition(0.3000, 0.3000)
            .setPosition(-0.3000, 0.3000)
            .setMoveMotor(RobotMap.CANBus.BACK_LEFT_MOVE, true, true)
            .setTurnMotor(RobotMap.CANBus.BACK_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
            .setLabel("Back Right")
            .useSparkMaxAttachedEncoder(0.0, false)
            .setPosition(-0.3000, -0.3000)
            .setMoveMotor(RobotMap.CANBus.BACK_RIGHT_MOVE, true, true)
            .setTurnMotor(RobotMap.CANBus.BACK_RIGHT_TURN, false, true);

        private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
            .setLabel("Front Right")
            .useSparkMaxAttachedEncoder(0.0, false)
            .setPosition(0.3000, -0.3000)
            .setMoveMotor(RobotMap.CANBus.FRONT_RIGHT_MOVE, true, true)
            .setTurnMotor(RobotMap.CANBus.FRONT_RIGHT_TURN, false, true);

        public static final SwerveConfig CONFIG = new SwerveConfig()
            .useADIS16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, SPI.Port.kOnboardCS0, CalibrationTime._4s)
            .setPeriod(PERIOD)
            .setMovePID(0.001, 0.0, 0.0)
            .setMoveFF(0.0, 2.84, 0.0)
            .setTurnPID(0.5, 0.0, 15.0)
            .setRampRate(0.03, 0.03)
            .setPowerProperties(VOLTAGE, 40.0, 30.0)
            .setMechanicalProperties(7.5, 10.0, 4.0)
            .setSpeedConstraints(4.0, 7.0, 8.0, 25.0)
            .setMotorTypes(SwerveMotorType.SPARK_MAX_BRUSHLESS, SwerveMotorType.SPARK_MAX_BRUSHLESS)
            .setDiscretizationLookahead(0.040)
            .setStandardDeviations(0.1, 0.1, 0.1)
            .setFieldSize(FIELD_LENGTH, FIELD_WIDTH)
            .addModule(FRONT_LEFT)
            .addModule(BACK_LEFT)
            .addModule(BACK_RIGHT)
            .addModule(FRONT_RIGHT);

        public static final double POSE_ROT_P = 7.0;
        public static final double POSE_ROT_I = 0.0;
        public static final double POSE_ROT_D = 0.5;
        public static final Constraints POSE_ROT_CONSTRAINTS = new Constraints(6.0, 12.5);
    }
}
