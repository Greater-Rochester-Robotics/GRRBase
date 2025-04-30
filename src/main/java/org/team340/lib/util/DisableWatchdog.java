package org.team340.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Watchdog;
import java.lang.reflect.Field;

/**
 * Utility class for disabling {@link Watchdog} instances inside objects.
 */
public final class DisableWatchdog {

    private DisableWatchdog() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * Disables a {@link Watchdog} instance within the specified object.
     * @param obj The object containing the {@link Watchdog} to disable.
     * @param fieldName The name of the field the {@link Watchdog} is declared as.
     */
    public static void in(Object obj, String fieldName) {
        try {
            Field field = null;
            Class<?> clazz = obj.getClass();

            while (field == null) {
                try {
                    field = clazz.getDeclaredField(fieldName);
                } catch (Exception e) {
                    clazz = clazz.getSuperclass();
                    if (clazz == null) throw new RuntimeException();
                }
            }

            field.setAccessible(true);
            Watchdog watchdog = (Watchdog) field.get(obj);
            watchdog.disable();

            watchdog.setTimeout(1e5);
        } catch (Exception e) {
            DriverStation.reportWarning(
                "Unable to disable watchdog: Attempted with accessor \"" +
                obj.getClass().getSimpleName() +
                "." +
                fieldName +
                "\"",
                false
            );
        }
    }
}
