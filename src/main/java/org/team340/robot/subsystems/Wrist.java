package org.team340.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.Math2;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

import com.reduxrobotics.frames.Frame;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * The wrist subsystem.
 * Responsible for pivoting the wrist.
 */
public class Wrist extends GRRSubsystem {
    // Limits
    public static final double MIN_POS = Math.toRadians(20.0);
    public static final double MAX_POS = Math.toRadians(140.0);

    // Positions
    public enum WristPosition {
        INTAKE(Math.toRadians(132.0)),
        SAFE(Math.toRadians(25.0)),
        SHOOT_SHORT(Math.toRadians(80.0)),
        SHOOT_MEDIUM(Math.toRadians(55.0)),
        SHOOT_FAR(Math.toRadians(45.0));

        public final double value;

        private WristPosition(double value) {
            this.value = value;
        }
    }

    // Misc
    public static final double CLOSED_LOOP_ERR = Math.toRadians(4.0);

    // Hardware Configs
    public static final class Configs {

        // Encoder Conversion Factor
        private static final double ENC_FACTOR = Math2.TWO_PI;

        public static final SparkMaxConfig MOTOR = new SparkMaxConfig()
            .clearFaults()
            .restoreFactoryDefaults()
            .enableVoltageCompensation(VOLTAGE)
            .setSmartCurrentLimit(30)
            .setIdleMode(IdleMode.kBrake)
            .setInverted(true)
            .setClosedLoopRampRate(0.3)
            .setOpenLoopRampRate(0.8)
            .setPeriodicFramePeriod(Frame.S0, 20)
            .setPeriodicFramePeriod(Frame.S1, 20)
            .setPeriodicFramePeriod(Frame.S2, 20)
            .setPeriodicFramePeriod(Frame.S3, 10000)
            .setPeriodicFramePeriod(Frame.S4, 10000)
            .setPeriodicFramePeriod(Frame.S5, 20)
            .setPeriodicFramePeriod(Frame.S6, 20);

        public static final SparkAbsoluteEncoderConfig ENCODER = new SparkAbsoluteEncoderConfig()
            .setPositionConversionFactor(ENC_FACTOR)
            .setVelocityConversionFactor(ENC_FACTOR / 60)
            .setInverted(true)
            .setZeroOffset(0.0);

        public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
            .setPID(1.85, 0.0, 0.3)
            .setIZone(0.0)
            .setPositionPIDWrappingEnabled(false);
    }
    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;

    private double maintain = 0.0;
    private boolean maintainEnabled = false;
    private double target = 0.0;

    public Wrist() {
        // motor = createSparkMax("Wrist NEO", Constants.RobotMap.WRIST_MOTOR, MotorType.kBrushless);
        // TODO: figure out what to do with this.
        motor = new SparkMax(RobotMap.kWristMotor, MotorType.kBrushless);
        // TODO: figure out this line.
        // encoder = createSparkMaxAbsoluteEncoder("Wrist Through Bore Encoder", motor, Type.kDutyCycle);
        pid = motor.get

        pid.setFeedbackDevice(encoder);

        WristConstants.Configs.MOTOR.apply(motor);
        WristConstants.Configs.ENCODER.apply(motor, encoder);
        WristConstants.Configs.PID.apply(motor, pid);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("maintain", () -> maintain, null);
        builder.addBooleanProperty("maintainEnabled", () -> maintainEnabled, null);
        builder.addDoubleProperty("target", () -> target, null);
    }

    /**
     * Returns {@code true} if the wrist is at the specified position.
     * @param position The position to check for in radians.
     */
    private boolean atPosition(WristPosition position) {
        return Math2.epsilonEquals(
            MathUtil.angleModulus(encoder.getPosition()),
            position.value,
            WristConstants.CLOSED_LOOP_ERR
        );
    }

    /**
     * Modifies the setpoint of the {@link #pid wrist's PID controller} to the specified position, if it is
     * within the {@link WristConstants#MIN_POS minimum} and {@link WristConstants#MAX_POS maximum} range.
     * @param position The position to set, in radians.
     */
    private void applyPosition(double position) {
        if (position < WristConstants.MIN_POS || position > WristConstants.MAX_POS) {
            DriverStation.reportWarning(
                "Invalid wrist position. " +
                Math2.formatRadians(position) +
                " degrees is not between " +
                Math2.formatRadians(WristConstants.MIN_POS) +
                " and " +
                Math2.formatRadians(WristConstants.MAX_POS),
                false
            );
        } else {
            target = position;
            maintainEnabled = true;
            pid.setReference(position, ControlType.kPosition);
        }
    }

    /**
     * Goes to a position. Ends after the position is reached.
     * @param position The position for the wrist to move to.
     */
    public Command goTo(WristPosition position) {
        return goTo(position, true);
    }

    /**
     * Goes to a position.
     * @param position The position for the wrist to move to.
     * @param willFinish If {@code true}, the command will end after the position is reached.
     */
    public Command goTo(WristPosition position, boolean willFinish) {
        return commandBuilder("wrist.toPosition(" + Math2.formatRadians(position.value) + ", " + willFinish + ")")
            .onExecute(() -> {
                applyPosition(position.value);
                maintain = encoder.getPosition();
            })
            .isFinished(() -> willFinish && atPosition(position))
            .onEnd(interrupted -> {
                if (!interrupted || atPosition(position)) maintain = position.value;
                motor.stopMotor();
            });
    }

    /**
     * Maintains the last set position.
     */
    public Command maintainPosition() {
        return commandBuilder("intake.maintainPosition()")
            .onExecute(() -> {
                if (maintainEnabled) applyPosition(maintain);
            })
            .onEnd(() -> motor.stopMotor());
    }

    /**
     * Should be ran while disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                motor.setIdleMode(IdleMode.kCoast);
                motor.
                motor.stopMotor();
            })
            .onExecute(() -> maintain = encoder.getPosition())
            .onEnd(() -> motor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("wrist.onDisable()");
    }
}
