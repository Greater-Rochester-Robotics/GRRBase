package org.team340.lib.util.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Utilities for the CTRE Phoenix 6 API.
 */
public final class PhoenixUtil {

    private PhoenixUtil() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * Runs a Phoenix API call and checks for errors. Will
     * retry up to 2 times if the target API call fails.
     * @param device The device the call is relevant to.
     * @param name The name of the API call.
     * @param target The target call to run.
     * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
     */
    public static boolean run(ParentDevice device, String name, Supplier<StatusCode> target) {
        return run(device, name, target, 2);
    }

    /**
     * Runs a Phoenix API call and checks for errors.
     * @param device The device the call is relevant to.
     * @param name The name of the API call.
     * @param target The target call to run.
     * @param retries The number of times to retry the call before failing. {@code 0} only runs the call once.
     * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
     */
    public static boolean run(ParentDevice device, String name, Supplier<StatusCode> target, int retries) {
        String results = "";
        for (int i = 0; i <= retries; i++) {
            StatusCode result = target.get();
            if (result.isOK()) return true;
            results += (results.isEmpty() ? "" : ", ") + result.name();
        }

        DriverStation.reportError(
            "[PhoenixUtil] " +
            device.getClass().getSimpleName() +
            " (ID " +
            device.getDeviceID() +
            ") \"" +
            name +
            "\": " +
            results,
            false
        );
        return false;
    }
}
