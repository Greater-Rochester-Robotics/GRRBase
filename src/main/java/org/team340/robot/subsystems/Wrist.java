package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.RevUtil;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The wrist subsystem.
 * Responsible for pivoting the wrist.
 */
@Logged
public class Wrist extends GRRSubsystem {

    // Limits
    private static final double kMinPos = 0.024;
    private static final double kMaxPos = 0.337;

    // Positions
    public enum Position {
        kIntake(0.337),
        kSafe(0.03),
        kShootShort(0.222222222222222),
        kShootMedium(0.152777777777778),
        kShootFar(0.125);

        public final TunableDouble kRotations;

        private Position(double rotations) {
            kRotations = Tunable.doubleValue("Wrist/Position/" + name(), rotations);
        }

        public double getRotations() {
            return kRotations.value();
        }
    }

    // Misc
    private static final double CLOSED_LOOP_ERR = 0.0111111111111111;

    private final SparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkClosedLoopController pid;

    private double maintain = 0.0;
    private boolean maintainEnabled = false;

    // This is logged by epilog.
    @SuppressWarnings("unused")
    private double target = 0.0;

    public Wrist() {
        motor = new SparkMax(RobotMap.kWristMotor, MotorType.kBrushless);

        encoder = motor.getAbsoluteEncoder();
        motor.clearFaults();

        pid = motor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();

        config
            .voltageCompensation(Constants.kVoltage)
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .closedLoopRampRate(0.3)
            .openLoopRampRate(0.8);

        config.signals
            .absoluteEncoderPositionPeriodMs(20)
            .absoluteEncoderVelocityPeriodMs(20)
            .analogPositionPeriodMs(20)
            .analogVelocityPeriodMs(20)
            .analogVoltagePeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .faultsPeriodMs(250)
            .iAccumulationPeriodMs(20)
            .limitsPeriodMs(20)
            .motorTemperaturePeriodMs(1000)
            .outputCurrentPeriodMs(20)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(20)
            .warningsPeriodMs(250);

        config.absoluteEncoder.velocityConversionFactor(1.0 / 60.0).inverted(true).zeroOffset(0.8);

        config.closedLoop
            .pid(7.0, 0.0, 0.3)
            .iZone(0.0)
            .positionWrappingEnabled(false)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

        RevUtil.config(motor, config);

        Tunable.pidController("Wrist/pid", motor);
    }

    // *************** Helper Functions ***************

    /**
     * Stops the Wrist
     */
    private void stop() {
        motor.stopMotor();
    }

    /**
     * Returns {@code true} if the wrist is at the specified position.
     * @param position The position to check for in radians.
     */
    private boolean atPosition(Position position) {
        return Math2.epsilonEquals(
            MathUtil.inputModulus(encoder.getPosition(), -0.5, 0.5),
            position.getRotations(),
            CLOSED_LOOP_ERR
        );
    }

    /**
     * Modifies the setpoint of the {@link #pid wrist's PID controller} to the specified position, if it is
     * within the {@link WristConstants#MIN_POS minimum} and {@link WristConstants#kMaxPos maximum} range.
     * @param position The position to set, in rotations.
     */
    private void applyPosition(double rotations) {
        if (rotations < kMinPos || rotations > kMaxPos) {
            DriverStation.reportWarning(
                "Invalid wrist rotations. " +
                rotations +
                " rotations is not between the minimum position (" +
                kMinPos +
                " rotations) and the maximum position (" +
                kMaxPos +
                " rotations).",
                false
            );
        } else {
            target = rotations;
            maintainEnabled = true;
            pid.setReference(rotations, ControlType.kPosition);
        }
    }

    // *************** Commands ***************

    /**
     * Goes to a position. Ends after the position is reached.
     * @param rotations The position for the wrist to move to.
     */
    public Command goTo(Position rotations) {
        return goTo(rotations, true).withName("Wrist.goTo(" + rotations.name() + ")");
    }

    /**
     * Moves the wrist to predetermined positions.
     * @param rotations The position for the wrist to move to.
     * @param willFinish If {@code true}, the command will end after the position is reached.
     */
    public Command goTo(Position rotations, boolean willFinish) {
        return commandBuilder("wrist.toPosition(" + rotations.name() + ", " + willFinish + ")")
            .onExecute(() -> {
                applyPosition(rotations.getRotations());
            })
            .isFinished(willFinish ? () -> atPosition(rotations) : () -> false)
            .onEnd(interrupted -> {
                stop();
                if (!interrupted || atPosition(rotations)) {
                    maintain = rotations.getRotations();
                } else {
                    maintain = encoder.getPosition();
                }
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
            .onEnd(this::stop);
    }

    /**
     * Should be ran while disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        final SparkMaxConfig kBreakMode = new SparkMaxConfig();
        kBreakMode.idleMode(IdleMode.kBrake);

        final SparkMaxConfig kCoastMode = new SparkMaxConfig();
        kCoastMode.idleMode(IdleMode.kCoast);

        return commandBuilder()
            .onInitialize(() -> {
                RevUtil.configEphemeral(motor, kCoastMode);
                motor.stopMotor();
            })
            .onExecute(() -> maintain = encoder.getPosition())
            .onEnd(() -> RevUtil.configEphemeral(motor, kBreakMode))
            .ignoringDisable(true)
            .withName("wrist.onDisable()");
    }
}
