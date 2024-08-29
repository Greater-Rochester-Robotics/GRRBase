package org.team340.lib.util;

import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
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
    public static void measure(Measure<Time> time) {
        ms(time.in(Milliseconds));
    }

    /**
     * Sleeps the thread for the specified duration in seconds.
     * @param seconds The time to sleep for in seconds.
     */
    public static void seconds(double seconds) {
        ms(seconds * 1000.0);
    }

    /**
     * Sleeps the thread for the specified duration in milliseconds.
     * @param ms The time to sleep for in milliseconds.
     */
    public static void ms(double ms) {
        if (!RobotBase.isSimulation()) {
            try {
                Thread.sleep((long) ms);
            } catch (Exception e) {}
        }
    }
}
