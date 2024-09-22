package org.team340.lib.util;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Utility class for sleeping the current thread, unless in simulation.
 */
public final class Sleep {

    private Sleep() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Sleeps the thread for the specified duration.
     * @param time The time to sleep for.
     */
    public static void of(Time time) {
        of(time, false);
    }

    /**
     * Sleeps the thread for the specified duration.
     * @param time The time to sleep for.
     * @param force If the thread should always sleep, even if the robot is in simulation.
     */
    public static void of(Time time, boolean force) {
        ms(time.in(Milliseconds), force);
    }

    /**
     * Sleeps the thread for the specified duration in seconds.
     * @param seconds The time to sleep for in seconds.
     */
    public static void seconds(double seconds) {
        seconds(seconds, false);
    }

    /**
     * Sleeps the thread for the specified duration in seconds.
     * @param seconds The time to sleep for in seconds.
     * @param force If the thread should always sleep, even if the robot is in simulation.
     */
    public static void seconds(double seconds, boolean force) {
        ms(seconds * 1000.0, force);
    }

    /**
     * Sleeps the thread for the specified duration in milliseconds.
     * @param ms The time to sleep for in milliseconds.
     */
    public static void ms(double ms) {
        ms(ms, false);
    }

    /**
     * Sleeps the thread for the specified duration in milliseconds.
     * @param ms The time to sleep for in milliseconds.
     * @param force If the thread should always sleep, even if the robot is in simulation.
     */
    public static void ms(double ms, boolean force) {
        if (force || !RobotBase.isSimulation()) {
            try {
                Thread.sleep((long) ms);
            } catch (Exception e) {}
        }
    }
}
