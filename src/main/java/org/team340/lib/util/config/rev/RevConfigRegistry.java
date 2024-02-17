package org.team340.lib.util.config.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import java.text.Collator;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;
import java.util.function.Supplier;
import org.team340.lib.util.Sleep;

/**
 * Utilities for REV hardware configs.
 */
public final class RevConfigRegistry {

    public static final double EPSILON = 1e-4;

    private static final double BURN_FLASH_START_SLEEP = 2000.0;
    private static final double BURN_FLASH_INTERVAL = 10.0;
    private static final double PERIODIC_INTERVAL = 500.0;

    private static final List<Runnable> periodicCallbacks = new ArrayList<>();
    private static final Map<String, Supplier<REVLibError>> burnFlashCallbacks = new LinkedHashMap<>();
    private static final Collection<String> errors = new TreeSet<>(ErrorComparator.getInstance());

    private RevConfigRegistry() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Saves a callback to a runnable for setting the periodic frame period.
     * @param callback The runnable.
     */
    static void addPeriodic(Runnable callback) {
        periodicCallbacks.add(callback);
    }

    /**
     * Saves a callback to burn flash to a Spark motor controller.
     * @param identifier The identifier of the Spark motor controller.
     * @param callback A callback that burns flash and returns the result.
     */
    static void addBurnFlash(String identifier, Supplier<REVLibError> callback) {
        burnFlashCallbacks.put(identifier, callback);
    }

    /**
     * Saves a configuration error string to be logged.
     * @param errorString The error string.
     */
    static void addError(String errorString) {
        errors.add(errorString);
    }

    /**
     * Initializes periodically applying frame periods from {@code periodicCallbacks} list.
     * @param robot Robot to call {@link TimedRobot#addPeriodic(Runnable, double) addPeriodic()} on.
     */
    public static void init(TimedRobot robot) {
        robot.addPeriodic(
            () -> {
                for (Runnable callback : periodicCallbacks) callback.run();
            },
            PERIODIC_INTERVAL
        );
    }

    /**
     * Burns flash to all registered Spark motor controllers.
     */
    public static void burnFlash() {
        Sleep.ms(BURN_FLASH_START_SLEEP);

        for (Map.Entry<String, Supplier<REVLibError>> entry : burnFlashCallbacks.entrySet()) {
            String result;
            try {
                result = entry.getValue().get().name();
                Sleep.ms(BURN_FLASH_INTERVAL);
            } catch (Exception e) {
                DriverStation.reportError(e.getMessage(), true);
                result = e.getClass().getSimpleName();
            }

            if (!result.equals(REVLibError.kOk.name())) addError(entry.getKey() + " \"Burn Flash\": " + result);
        }
    }

    /**
     * Prints unsuccessful configurations to stdout.
     * Useful for debugging, should be ran after initializing all hardware.
     */
    public static void printError() {
        if (errors.size() <= 0) {
            System.out.println("\nAll REV hardware configured successfully\n");
        } else {
            DriverStation.reportWarning("\nErrors while configuring " + errors.size() + " options on REV hardware:", false);

            for (String errorString : errors) {
                DriverStation.reportWarning("\t" + errorString, false);
            }
            DriverStation.reportWarning("\n", false);
            errors.clear();
        }
    }

    private static final class ErrorComparator implements Comparator<String> {

        private static ErrorComparator instance;
        private final Collator localeComparator = Collator.getInstance();

        public static ErrorComparator getInstance() {
            if (instance == null) instance = new ErrorComparator();
            return instance;
        }

        private ErrorComparator() {}

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
