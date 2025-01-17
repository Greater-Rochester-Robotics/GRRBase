package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants;
import org.team340.robot.Constants.WristConstants;
import org.team340.robot.Constants.WristConstants.WristPosition;

/**
 * The wrist subsystem.
 * Responsible for pivoting the wrist.
 */
public class Wrist extends GRRSubsystem {

    private final CANSparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController pid;

    private double maintain = 0.0;
    private boolean maintainEnabled = false;
    private double target = 0.0;

    public Wrist() {
        super("Wrist");
        motor = createSparkMax("Wrist NEO", Constants.RobotMap.WRIST_MOTOR, MotorType.kBrushless);
        encoder = createSparkMaxAbsoluteEncoder("Wrist Through Bore Encoder", motor, Type.kDutyCycle);
        pid = motor.getPIDController();

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
                motor.stopMotor();
            })
            .onExecute(() -> maintain = encoder.getPosition())
            .onEnd(() -> motor.setIdleMode(IdleMode.kBrake))
            .ignoringDisable(true)
            .withName("wrist.onDisable()");
    }
}
