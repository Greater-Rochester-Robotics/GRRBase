package org.team340.lib.dashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;

/**
 * The Tunable class is used to construct tunable properties of the
 * robot to be displayed and edited in the dashboard. A tunable must
 * be published via {@link GRRDashboard#addTunable(Tunable)} before it
 * is available in the dashboard. Pre-built tunables that wrap commonly
 * used interfaces are available via static methods of this class.
 */
public final class Tunable {

    static final record TunableField<T>(String name, NetworkTableType type, T defaultValue, Function<NetworkTable, Runnable> init) {}

    final List<TunableField<?>> fields = new ArrayList<>();
    final String name;

    boolean devOnly = false;

    /**
     * Creates a tunable.
     * @param name The name for the tunable. Must be unique.
     */
    public Tunable(String name) {
        this.name = name;
    }

    /**
     * If the tunable is "dev only", and will only be displayed
     * when the developer mode toggle is on in the dashboard.
     * @param devOnly {@link true} if the tunable should be dev only.
     */
    public Tunable devOnly(boolean devOnly) {
        this.devOnly = devOnly;
        return this;
    }

    /**
     * Adds an integer field.
     * @param name The name of the field. Must be unique.
     * @param defaultValue The default value of the field (e.g. the programmed value).
     * @param setter A setter that is called when a new value is received from the dashboard.
     */
    public Tunable addInteger(String name, int defaultValue, Consumer<Integer> setter) {
        fields.add(
            new TunableField<Integer>(
                name,
                NetworkTableType.kInteger,
                defaultValue,
                nt -> {
                    var sub = nt.getIntegerTopic(name).subscribe(defaultValue);
                    return () -> {
                        for (long v : sub.readQueueValues()) {
                            setter.accept((int) v);
                        }
                    };
                }
            )
        );
        return this;
    }

    /**
     * Adds a float field.
     * @param name The name of the field. Must be unique.
     * @param defaultValue The default value of the field (e.g. the programmed value).
     * @param setter A setter that is called when a new value is received from the dashboard.
     */
    public Tunable addFloat(String name, float defaultValue, Consumer<Float> setter) {
        fields.add(
            new TunableField<Float>(
                name,
                NetworkTableType.kFloat,
                defaultValue,
                nt -> {
                    var sub = nt.getFloatTopic(name).subscribe(defaultValue);
                    return () -> {
                        for (float v : sub.readQueueValues()) {
                            setter.accept(v);
                        }
                    };
                }
            )
        );
        return this;
    }

    /**
     * Adds a double field.
     * @param name The name of the field. Must be unique.
     * @param defaultValue The default value of the field (e.g. the programmed value).
     * @param setter A setter that is called when a new value is received from the dashboard.
     */
    public Tunable addDouble(String name, double defaultValue, Consumer<Double> setter) {
        fields.add(
            new TunableField<Double>(
                name,
                NetworkTableType.kDouble,
                defaultValue,
                nt -> {
                    var sub = nt.getDoubleTopic(name).subscribe(defaultValue);
                    return () -> {
                        for (double v : sub.readQueueValues()) {
                            setter.accept(v);
                        }
                    };
                }
            )
        );
        return this;
    }

    /**
     * Adds a boolean field.
     * @param name The name of the field. Must be unique.
     * @param defaultValue The default value of the field (e.g. the programmed value).
     * @param setter A setter that is called when a new value is received from the dashboard.
     */
    public Tunable addBoolean(String name, boolean defaultValue, Consumer<Boolean> setter) {
        fields.add(
            new TunableField<Boolean>(
                name,
                NetworkTableType.kBoolean,
                defaultValue,
                nt -> {
                    var sub = nt.getBooleanTopic(name).subscribe(defaultValue);
                    return () -> {
                        for (boolean v : sub.readQueueValues()) {
                            setter.accept(v);
                        }
                    };
                }
            )
        );
        return this;
    }

    /**
     * Adds a string field.
     * @param name The name of the field. Must be unique.
     * @param defaultValue The default value of the field (e.g. the programmed value).
     * @param setter A setter that is called when a new value is received from the dashboard.
     */
    public Tunable addString(String name, String defaultValue, Consumer<String> setter) {
        fields.add(
            new TunableField<String>(
                name,
                NetworkTableType.kString,
                defaultValue,
                nt -> {
                    var sub = nt.getStringTopic(name).subscribe(defaultValue);
                    return () -> {
                        for (String v : sub.readQueueValues()) {
                            setter.accept(v);
                        }
                    };
                }
            )
        );
        return this;
    }

    /**
     * A pre-built {@link Tunable} that wraps a WPILib {@link PIDController}.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static Tunable pidController(String name, PIDController controller) {
        return new Tunable(name)
            .devOnly(true)
            .addDouble("kP", controller.getP(), v -> controller.setP(v))
            .addDouble("kI", controller.getI(), v -> controller.setI(v))
            .addDouble("kD", controller.getD(), v -> controller.setD(v))
            .addDouble("iZone", controller.getIZone(), v -> controller.setIZone(v));
    }

    /**
     * A pre-built {@link Tunable} that wraps a WPILib {@link ProfiledPIDController}.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static Tunable pidController(String name, ProfiledPIDController controller) {
        return new Tunable(name)
            .devOnly(true)
            .addDouble("-kP", controller.getP(), v -> controller.setP(v))
            .addDouble("-kI", controller.getI(), v -> controller.setI(v))
            .addDouble("-kD", controller.getD(), v -> controller.setD(v))
            .addDouble("-iZone", controller.getIZone(), v -> controller.setIZone(v))
            .addDouble(
                "-maxV",
                controller.getConstraints().maxVelocity,
                v -> controller.setConstraints(new TrapezoidProfile.Constraints(v, controller.getConstraints().maxAcceleration))
            )
            .addDouble(
                "-maxA",
                controller.getConstraints().maxAcceleration,
                v -> controller.setConstraints(new TrapezoidProfile.Constraints(controller.getConstraints().maxVelocity, v))
            );
    }

    /**
     * A pre-built {@link Tunable} that wraps a {@link SparkPIDController REV Spark PID Controller}.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static Tunable pidController(String name, SparkPIDController controller) {
        return pidController(name, controller, 0);
    }

    /**
     * A pre-built {@link Tunable} that wraps a {@link SparkPIDController REV Spark PID Controller}.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     * @param slotId The slot of the PID controller to use.
     */
    public static Tunable pidController(String name, SparkPIDController controller, int slotId) {
        return new Tunable(name)
            .devOnly(true)
            .addDouble("-kP", controller.getP(slotId), v -> controller.setP(v, slotId))
            .addDouble("-kI", controller.getI(slotId), v -> controller.setI(v, slotId))
            .addDouble("-kD", controller.getD(slotId), v -> controller.setD(v, slotId))
            .addDouble("-iZone", controller.getIZone(slotId), v -> controller.setIZone(v, slotId))
            .addDouble("-kFF", controller.getFF(slotId), v -> controller.setFF(slotId));
    }

    /**
     * A pre-built {@link Tunable} that modifies gains of a {@link TalonFX} PID controller.
     * @param name The name for the tunable. Must be unique.
     * @param controller The PID controller.
     */
    public static Tunable pidController(String name, TalonFX controller) {
        Slot0Configs config = new Slot0Configs();
        controller.getConfigurator().refresh(config);

        return new Tunable(name)
            .devOnly(true)
            .addDouble(
                "-kP",
                config.kP,
                v -> {
                    config.kP = v;
                    controller.getConfigurator().apply(config);
                }
            )
            .addDouble(
                "-kI",
                config.kI,
                v -> {
                    config.kI = v;
                    controller.getConfigurator().apply(config);
                }
            )
            .addDouble(
                "-kD",
                config.kD,
                v -> {
                    config.kD = v;
                    controller.getConfigurator().apply(config);
                }
            )
            .addDouble(
                "-kS",
                config.kS,
                v -> {
                    config.kS = v;
                    controller.getConfigurator().apply(config);
                }
            )
            .addDouble(
                "-kV",
                config.kV,
                v -> {
                    config.kV = v;
                    controller.getConfigurator().apply(config);
                }
            )
            .addDouble(
                "-kA",
                config.kA,
                v -> {
                    config.kA = v;
                    controller.getConfigurator().apply(config);
                }
            )
            .addDouble(
                "-kG",
                config.kG,
                v -> {
                    config.kG = v;
                    controller.getConfigurator().apply(config);
                }
            );
    }
}
