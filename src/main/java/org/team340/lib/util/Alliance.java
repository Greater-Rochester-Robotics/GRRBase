package org.team340.lib.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class for getting the robot's alliance.
 * Can optionally be overridden using {@link Alliance#enableOverride(boolean)}.
 */
public final class Alliance {

    private Alliance() {
        throw new AssertionError("This is a utility class!");
    }

    private static boolean overrideActive = false;
    private static boolean overrideIsBlue = false;

    /**
     * Overrides FMS alliance data in favor of a user-set value when
     * using {@link Alliance#isBlue()} and {@link Alliance#isRed()}.
     * @param isBlue The value to override with.
     */
    public static void enableOverride(boolean isBlue) {
        overrideActive = true;
        overrideIsBlue = isBlue;
    }

    /**
     * Disables the override if active.
     */
    public static void disableOverride() {
        overrideActive = false;
    }

    /**
     * Returns {@code true} if the robot is on the blue alliance.
     * If the robot's alliance is unknown, defaults to {@code true} (blue).
     */
    public static boolean isBlue() {
        return overrideActive
            ? overrideIsBlue
            : DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Blue);
    }

    /**
     * Returns {@code true} if the robot is on the red alliance.
     * If the robot's alliance is unknown, defaults to {@code false} (blue).
     */
    public static boolean isRed() {
        return !isBlue();
    }
}
