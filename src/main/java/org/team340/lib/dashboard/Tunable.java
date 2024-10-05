package org.team340.lib.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Subscriber;
import java.util.function.Consumer;
import java.util.function.Supplier;

public final class Tunable<T> implements Supplier<T> {

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("GRRDashboard/Tunables");

    private final Supplier<T> getter;
    private final Subscriber sub;
    private long lastSet;

    private Tunable(Subscriber sub, Supplier<T> getter, Consumer<T> onChange) {
        this.getter = getter;
        this.sub = sub;

        if (onChange != null) {
            GRRDashboard.bind(() -> {
                long lastChange = getLastChange();
                if (lastChange != lastSet) {
                    lastSet = lastChange;
                    onChange.accept(get());
                }
            });
        }
    }

    /**
     * Gets the current value of the tunable.
     */
    @Override
    public T get() {
        return getter.get();
    }

    /**
     * Gets the last time the entry's value was changed.
     */
    public long getLastChange() {
        return sub.getLastChange();
    }

    /**
     * Creates a tunable integer.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static Tunable<Integer> integer(String name, int defaultValue) {
        return integer(name, defaultValue, null);
    }

    /**
     * Creates a tunable integer.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static Tunable<Integer> integer(String name, int defaultValue, Consumer<Integer> onChange) {
        var entry = nt.getIntegerTopic(name).getEntry(defaultValue);
        entry.setDefault(defaultValue);
        return new Tunable<>(entry, () -> (int) entry.get(), onChange);
    }
}
