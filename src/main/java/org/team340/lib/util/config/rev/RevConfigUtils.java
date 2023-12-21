package org.team340.lib.util.config.rev;

import edu.wpi.first.wpilibj.RobotBase;
import java.text.Collator;
import java.util.Collection;
import java.util.Comparator;
import java.util.TreeSet;

/**
 * Utilities for REV hardware configs.
 */
public final class RevConfigUtils {

    public static final double EPSILON = 1e-4;
    public static final double BURN_FLASH_SLEEP = 250.0;
    public static final double CHECK_SLEEP = 25;
    public static final int SET_ITERATIONS = 3;

    private static final Collection<String> success = new TreeSet<>(SuccessComparator.getInstance());

    private RevConfigUtils() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Blocks the thread to ensure a safe burn to flash.
     */
    static void burnFlashSleep() {
        if (!RobotBase.isSimulation()) {
            try {
                Thread.sleep((long) BURN_FLASH_SLEEP);
            } catch (Exception e) {}
        }
    }

    /**
     * Saves a configuration success string to be logged.
     * @param successString The success string.
     */
    static void addSuccess(String successString) {
        success.add(successString);
    }

    /**
     * Prints successful configurations to stdout.
     * Useful for debugging, should be ran after initializing all hardware.
     */
    public static void printSuccess() {
        System.out.println("\nSuccessfully configured " + success.size() + " options on REV hardware:");
        for (String successString : success) {
            System.out.println("\t" + successString);
        }
        System.out.println("\n");
        success.clear();
    }

    private static final class SuccessComparator implements Comparator<String> {

        private static SuccessComparator instance;
        private final Collator localeComparator = Collator.getInstance();

        public static SuccessComparator getInstance() {
            if (instance == null) instance = new SuccessComparator();
            return instance;
        }

        private SuccessComparator() {}

        @Override
        public int compare(String arg0, String arg1) {
            int idDiff;
            try {
                idDiff = Integer.parseInt(arg0.split(" ")[1]) - Integer.parseInt(arg1.split(" ")[1]);
            } catch (Exception e) {
                idDiff = 0;
            }

            if (idDiff == 0) {
                int localeDiff = localeComparator.compare(arg0, arg1);
                if (localeDiff == 0) return 1; else return localeDiff;
            } else {
                return idDiff;
            }
        }
    }
}
