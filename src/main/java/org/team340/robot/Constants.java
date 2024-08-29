package org.team340.robot;

import org.team340.lib.controller.Controller2Config;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double PERIOD = 0.020;
    public static final double TELEMETRY_PERIOD = 0.020;
    public static final double POWER_USAGE_PERIOD = 0.020;
    public static final double VOLTAGE = 12.0;
    public static final double FIELD_LENGTH = 16.541;
    public static final double FIELD_WIDTH = 8.211;

    /**
     * Driver and co-driver controller constants.
     */
    public static final class ControllerConstants {

        public static final double DRIVE_EXP = 2.0;
        public static final double DRIVE_MULTIPLIER = 0.9;
        public static final double DRIVE_MULTIPLIER_MODIFIED = 1.0;

        public static final double DRIVE_ROT_EXP = 3.0;
        public static final double DRIVE_ROT_MULTIPLIER = 0.4;

        public static final Controller2Config DRIVER = new Controller2Config()
            .setLabel("Driver")
            .setPort(0)
            .setJoystickDeadband(0.1)
            .setJoystickThreshold(0.5)
            .setTriggerDeadband(0.05)
            .setTriggerThreshold(0.05);

        public static final Controller2Config CO_DRIVER = new Controller2Config()
            .setLabel("CoDriver")
            .setPort(1)
            .setJoystickDeadband(0.1)
            .setJoystickThreshold(0.5)
            .setTriggerDeadband(0.05)
            .setTriggerThreshold(0.05);
    }

    /**
     * Map of hardware device IDs.
     */
    public static final class RobotMap {}
}
