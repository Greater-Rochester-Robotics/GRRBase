package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
    private static final double kMinPos = Math.toRadians(20.0);
    private static final double kMaxPos = Math.toRadians(140.0);

    // Positions
    public enum WristPosition {
        kIntake(Math.toRadians(132.0)),
        kSafe(Math.toRadians(25.0)),
        kShootShort(Math.toRadians(80.0)),
        kShootMedium(Math.toRadians(55.0)),
        kShootFar(Math.toRadians(45.0));

        public final TunableDouble position;

        private WristPosition(double position) {
            this.position = Tunable.doubleValue("wristPosition " + name().substring(1), position);
        }

        public double getPosition() {
            return position.value();
        }
    }

    // Misc
    private static final double CLOSED_LOOP_ERR = Math.toRadians(4.0);

    // Encoder Conversion Factor
    private static final double ENC_FACTOR = Math.PI * 2;

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

        config.absoluteEncoder
            .positionConversionFactor(ENC_FACTOR)
            .velocityConversionFactor(ENC_FACTOR / 60.0)
            .inverted(true)
            .zeroOffset(0.8);

        config.closedLoop.pid(1.85, 0.0, 0.3).iZone(0.0).positionWrappingEnabled(false);

        RevUtil.config(motor, config);

        Tunable.pidController("Wrist/pid", motor);
    }

    /**
     * Returns {@code true} if the wrist is at the specified position.
     * @param position The position to check for in radians.
     */
    private boolean atPosition(WristPosition position) {
        return Math2.epsilonEquals(
            MathUtil.angleModulus(encoder.getPosition()),
            position.getPosition(),
            CLOSED_LOOP_ERR
        );
    }

    /**
     * Modifies the setpoint of the {@link #pid wrist's PID controller} to the specified position, if it is
     * within the {@link WristConstants#MIN_POS minimum} and {@link WristConstants#kMaxPos maximum} range.
     * @param position The position to set, in radians.
     */
    private void applyPosition(double position) {
        if (position < kMinPos || position > kMaxPos) {
            DriverStation.reportWarning(
                "Invalid wrist position. " +
                position +
                " radians is not between the minimum position (" +
                kMinPos +
                " radians) and the maximum position (" +
                kMaxPos +
                " radians).",
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
        return commandBuilder("wrist.toPosition(" + position.name() + ", " + willFinish + ")")
            .onExecute(() -> {
                applyPosition(position.getPosition());
            })
            .isFinished(willFinish ? () -> atPosition(position) : () -> false)
            .onEnd(interrupted -> {
                if (!interrupted || atPosition(position)) {
                    maintain = position.getPosition();
                } else {
                    maintain = encoder.getPosition();
                }
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
