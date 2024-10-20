package org.team340.lib.util.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import java.text.Collator;
import java.util.Collection;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;
import org.team340.lib.util.Sleep;

/**
 * Utilities for REV hardware configs.
 */
public final class RevConfigRegistry {

    private RevConfigRegistry() {
        throw new AssertionError("This is a utility class!");
    }

    static final double EPSILON = 1e-3;

    private static final double BURN_FLASH_DELAY = 1.0;
    private static final double BURN_FLASH_INTERVAL = 0.075;

    private static final Collection<String> errors = new TreeSet<>(ErrorComparator.getInstance());
    private static final Map<String, Runnable> frameRefreshers = new ConcurrentHashMap<>();
    private static final Map<String, Supplier<REVLibError>> burnFlash = new LinkedHashMap<>();

    private static Thread frameThread = null;
    private static volatile boolean frameThreadActive = false;

    /**
     * Saves a configuration error string to be logged.
     * @param errorString The error string.
     */
    static void addError(String errorString) {
        errors.add(errorString);
    }

    /**
     * Saves a callback to be called periodically to refresh a device's CAN frame period setting.
     * All refreshers should be associated with a unique identifier, that is relevant to the device
     * and specific frame being refreshed, to ensure if a device has a frame configured twice only
     * the last configured period is utilized while refreshing.
     * @param identifier An identifier for the refresher.
     * @param refresher A {@link Runnable} to invoke.
     */
    static void addFrameRefresher(String identifier, Runnable refresher) {
        frameRefreshers.put(identifier, refresher);
    }

    /**
     * Saves a callback to burn flash to REV hardware.
     * @param identifier An readable identifier for the hardware. Must be unique.
     * @param callback A callback that burns flash and returns the result.
     */
    static void addBurnFlash(String identifier, Supplier<REVLibError> callback) {
        burnFlash.put(identifier, callback);
    }

    /**
     * Burns flash to all registered REV hardware. Additionally, any
     * errors accumulated while previously configuring hardware will
     * be printed to stdout. Note that this is a blocking operation.
     */
    public static void burnFlashAll() {
        Sleep.seconds(BURN_FLASH_DELAY, true);

        for (var entry : burnFlash.entrySet()) {
            REVLibError result = entry.getValue().get();
            Sleep.seconds(BURN_FLASH_INTERVAL, true);
            if (!result.equals(REVLibError.kOk)) addError(entry.getKey() + " \"Burn Flash\": " + result.name());
        }

        if (errors.isEmpty()) {
            System.out.println("\n[RevConfigRegistry] All REV hardware configured successfully\n");
        } else {
            String error = "[RevConfigRegistry] " + errors.size() + " errors while configuring REV hardware:";
            for (String e : errors) {
                error += "\n\t" + e;
            }
            DriverStation.reportError(error, false);
            errors.clear();
        }
    }

    /**
     * Enables refreshing device CAN frame period settings, in the case of a reset.
     * Frames are refreshed sequentially, and delayed evenly across the default
     * period of 10 seconds to prevent the CAN output buffer overflowing.
     */
    public static void enableFrameRefreshing() {
        enableFrameRefreshing(10.0);
    }

    /**
     * Enables refreshing device CAN frame period settings, in the case of a reset.
     * Frames are refreshed sequentially, and delayed evenly across the specified
     * period to prevent the CAN output buffer overflowing.
     * @param period The period at which to refresh a single frame at in seconds.
     */
    public static void enableFrameRefreshing(double period) {
        if (!frameThreadActive) {
            frameThread = new Thread(() -> {
                while (frameThreadActive) {
                    if (frameRefreshers.size() == 0) {
                        Sleep.seconds(period);
                        continue;
                    }

                    double delay = period / frameRefreshers.size();
                    for (Runnable callback : frameRefreshers.values()) {
                        callback.run();
                        Sleep.seconds(delay);
                    }
                }
            });

            frameThread.setName("RevConfigRegistry");
            frameThread.setDaemon(true);
            frameThreadActive = true;
            frameThread.start();
        }
    }

    /**
     * Disables refreshing device CAN frame period
     * settings, if previously enabled.
     */
    public static void disableFrameRefreshing() {
        if (frameThreadActive) {
            frameThreadActive = false;
            if (frameThread != null && frameThread.isAlive()) {
                try {
                    frameThread.interrupt();
                    frameThread.join();
                } catch (Exception e) {
                    Thread.currentThread().interrupt();
                }
            }
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
                if (localeDiff == 0) return 1;
                else return localeDiff;
            } else {
                return idDiff;
            }
        }
    }
}
