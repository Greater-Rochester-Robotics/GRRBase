package org.team340.lib.dashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Subscriber;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * The Tunable class is used to construct tunable properties of the robot to be modified
 * via NetworkTables, as well as automatically displayed and edited in the dashboard.
 */
public final class Tunable<T> implements Supplier<T>, AutoCloseable {

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("GRRDashboard/Tunables");

    private final Subscriber sub;
    private final Supplier<T> getter;
    private long lastSet;

    private Tunable(Subscriber sub, Supplier<T> getter, Consumer<T> onChange) {
        this.sub = sub;
        this.getter = getter;

        if (onChange != null) {
            GRRDashboard.bind(() -> {
                long lastChange = sub.getLastChange();
                if (lastChange != lastSet) {
                    lastSet = lastChange;
                    onChange.accept(getter.get());
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

    @Override
    public void close() {
        sub.close();
    }

    /**
     * Creates a tunable boolean.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static Tunable<Boolean> booleanValue(String name, boolean defaultValue) {
        return booleanValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable boolean.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static Tunable<Boolean> booleanValue(String name, boolean defaultValue, Consumer<Boolean> onChange) {
        var entry = nt.getBooleanTopic(name).getEntry(defaultValue);
        entry.setDefault(defaultValue);
        return new Tunable<>(entry, () -> entry.get(), onChange);
    }

    /**
     * Creates a tunable integer.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static Tunable<Integer> intValue(String name, int defaultValue) {
        return intValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable integer.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static Tunable<Integer> intValue(String name, int defaultValue, Consumer<Integer> onChange) {
        var entry = nt.getIntegerTopic(name).getEntry(defaultValue);
        entry.setDefault(defaultValue);
        return new Tunable<>(entry, () -> (int) entry.get(), onChange);
    }

    /**
     * Creates a tunable float.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static Tunable<Float> floatValue(String name, float defaultValue) {
        return floatValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable float.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static Tunable<Float> floatValue(String name, float defaultValue, Consumer<Float> onChange) {
        var entry = nt.getFloatTopic(name).getEntry(defaultValue);
        entry.setDefault(defaultValue);
        return new Tunable<>(entry, () -> entry.get(), onChange);
    }

    /**
     * Creates a tunable double.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static Tunable<Double> doubleValue(String name, double defaultValue) {
        return doubleValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable double.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static Tunable<Double> doubleValue(String name, double defaultValue, Consumer<Double> onChange) {
        var entry = nt.getDoubleTopic(name).getEntry(defaultValue);
        entry.setDefault(defaultValue);
        return new Tunable<>(entry, () -> entry.get(), onChange);
    }

    /**
     * Creates a tunable string.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     */
    public static Tunable<String> stringValue(String name, String defaultValue) {
        return stringValue(name, defaultValue, null);
    }

    /**
     * Creates a tunable string.
     * @param name The name for the tunable. Must be unique.
     * @param defaultValue The default value of the tunable (e.g. a programmed constant).
     * @param onChange A consumer that is invoked when the value of the tunable is modified.
     */
    public static Tunable<String> stringValue(String name, String defaultValue, Consumer<String> onChange) {
        var entry = nt.getStringTopic(name).getEntry(defaultValue);
        entry.setDefault(defaultValue);
        return new Tunable<>(entry, () -> entry.get(), onChange);
    }

    /**
     * Wraps a WPILib {@link PIDController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static void pidController(String name, PIDController controller) {
        doubleValue(name + "/kP", controller.getP(), v -> controller.setP(v));
        doubleValue(name + "/kI", controller.getI(), v -> controller.setI(v));
        doubleValue(name + "/kD", controller.getD(), v -> controller.setD(v));
        doubleValue(name + "/iZone", controller.getIZone(), v -> controller.setIZone(v));
    }

    /**
     * Wraps a WPILib {@link ProfiledPIDController} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static void pidController(String name, ProfiledPIDController controller) {
        doubleValue(name + "/kP", controller.getP(), v -> controller.setP(v));
        doubleValue(name + "/kI", controller.getI(), v -> controller.setI(v));
        doubleValue(name + "/kD", controller.getD(), v -> controller.setD(v));
        doubleValue(name + "/iZone", controller.getIZone(), v -> controller.setIZone(v));
        doubleValue(name + "/maxVelocity", controller.getConstraints().maxVelocity, v ->
            controller.setConstraints(new TrapezoidProfile.Constraints(v, controller.getConstraints().maxAcceleration))
        );
        doubleValue(name + "/maxAcceleration", controller.getConstraints().maxAcceleration, v ->
            controller.setConstraints(new TrapezoidProfile.Constraints(controller.getConstraints().maxVelocity, v))
        );
    }

    /**
     * Wraps a {@link SparkPIDController REV Spark PID Controller} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static void pidController(String name, SparkPIDController controller) {
        pidController(name, controller, 0);
    }

    /**
     * Wraps a {@link SparkPIDController REV Spark PID Controller} to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     * @param slotId The slot of the PID controller to use.
     */
    public static void pidController(String name, SparkPIDController controller, int slotId) {
        doubleValue(name + "/kP", controller.getP(slotId), v -> controller.setP(v, slotId));
        doubleValue(name + "/kI", controller.getI(slotId), v -> controller.setI(v, slotId));
        doubleValue(name + "/kD", controller.getD(slotId), v -> controller.setD(v, slotId));
        doubleValue(name + "/iZone", controller.getIZone(slotId), v -> controller.setIZone(v, slotId));
        doubleValue(name + "/kFF", controller.getFF(slotId), v -> controller.setFF(slotId));
    }

    /**
     * Wraps a {@link TalonFX} PID controller to be tunable.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static void pidController(String name, TalonFX controller) {
        Slot0Configs config = new Slot0Configs();
        controller.getConfigurator().refresh(config);

        doubleValue(name + "/kP", config.kP, v -> {
            config.kP = v;
            controller.getConfigurator().apply(config);
        });
        doubleValue(name + "/kI", config.kI, v -> {
            config.kI = v;
            controller.getConfigurator().apply(config);
        });
        doubleValue(name + "/kD", config.kD, v -> {
            config.kD = v;
            controller.getConfigurator().apply(config);
        });
        doubleValue(name + "/kS", config.kS, v -> {
            config.kS = v;
            controller.getConfigurator().apply(config);
        });
        doubleValue(name + "/kV", config.kV, v -> {
            config.kV = v;
            controller.getConfigurator().apply(config);
        });
        doubleValue(name + "/kA", config.kA, v -> {
            config.kA = v;
            controller.getConfigurator().apply(config);
        });
        doubleValue(name + "/kG", config.kG, v -> {
            config.kG = v;
            controller.getConfigurator().apply(config);
        });
    }
}
