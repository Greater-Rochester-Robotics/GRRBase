package org.team340.lib.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.Math2;

/**
 * A swerve module for {@link SwerveBase}
 */
public class SwerveModule {

    private final SwerveConfig config;
    private final SwerveModuleConfig moduleConfig;
    private final SwerveMotor moveMotor;
    private final SwerveMotor turnMotor;
    private final SwerveEncoder encoder;
    private final SimpleMotorFeedforward moveFF;
    private final Timer controlTimer = new Timer();

    private SwerveModuleState desiredState = new SwerveModuleState();
    private double lastMoveSpeed = 0.0;
    private double simDistance = 0.0;
    private double simHeading = 0.0;
    private double simVelocity = 0.0;

    /**
     * Create the swerve module.
     * @param moveMotor The module's move motor.
     * @param turnMotor The module's turn motor.
     * @param encoder The module's encoder.
     * @param config The general swerve config.
     * @param moduleConfig The module's config.
     */
    SwerveModule(
        SwerveMotor moveMotor,
        SwerveMotor turnMotor,
        SwerveEncoder encoder,
        SwerveConfig config,
        SwerveModuleConfig moduleConfig
    ) {
        this.config = config;
        this.moduleConfig = moduleConfig;
        this.moveMotor = moveMotor;
        this.turnMotor = turnMotor;
        this.encoder = encoder;

        moveFF = config.getMoveFF().simpleMotorFeedForward();
    }

    /**
     * Configures the current limit of the move motor.
     * @param newLimit The new current limit.
     */
    public void configMoveCurrentLimit(double newLimit) {
        moveMotor.configCurrentLimit(newLimit);
    }

    /**
     * Configures the current limit of the turn motor.
     * @param newLimit The new current limit.
     */
    public void configTurnCurrentLimit(double newLimit) {
        turnMotor.configCurrentLimit(newLimit);
    }

    /**
     * Gets the module's label.
     */
    public String getLabel() {
        return moduleConfig.getLabel();
    }

    /**
     * Gets current duty cycle of move motor.
     */
    public double getMoveDutyCycle() {
        if (RobotBase.isSimulation()) {
            return (simVelocity / config.getVelocity()) * 12.0;
        } else {
            return moveMotor.getDutyCycle();
        }
    }

    /**
     * Gets the velocity of the swerve module in meters/second.
     */
    public double getVelocity() {
        if (RobotBase.isSimulation()) {
            return simVelocity;
        } else {
            return moveMotor.getVelocity();
        }
    }

    /**
     * Gets the distance traveled by the swerve module in meters.
     */
    public double getDistance() {
        if (RobotBase.isSimulation()) {
            return simDistance;
        } else {
            return moveMotor.getPosition();
        }
    }

    /**
     * Gets the heading of the swerve module in radians.
     */
    public double getHeading() {
        if (RobotBase.isSimulation()) {
            return simHeading;
        } else {
            return encoder.getPosition();
        }
    }

    /**
     * Gets the desired state of the swerve module.
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Gets the current state of the swerve module.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromRadians(getHeading()));
    }

    /**
     * Gets the current position of the swerve module.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromRadians(getHeading()));
    }

    /**
     * Sets the desired state of the swerve module.
     * @param state The new state.
     */
    public void setDesiredState(SwerveModuleState state) {
        double moveSpeed = state.speedMetersPerSecond;
        double angleDiff = state.angle.rotateBy(Rotation2d.fromRadians(getHeading()).times(-1.0)).getRadians();
        boolean flipped = false;
        if (Math.abs(angleDiff) > (Math2.HALF_PI)) {
            if (angleDiff > 0) angleDiff -= Math.PI; else angleDiff += Math.PI;
            moveSpeed *= -1.0;
            flipped = true;
        }

        moveMotor.setReference(moveSpeed, moveFF.calculate(moveSpeed));
        turnMotor.setReference(turnMotor.getPosition() + angleDiff, 0.0);

        if (RobotBase.isSimulation()) {
            simDistance += simVelocity * controlTimer.get();
            simHeading =
                Math.signum(moveSpeed) == Math.signum(state.speedMetersPerSecond)
                    ? state.angle.getRadians()
                    : state.angle.minus(Math2.ROTATION2D_PI).getRadians();
            simVelocity = moveSpeed;
        }

        desiredState = flipped ? new SwerveModuleState(-state.speedMetersPerSecond, state.angle.rotateBy(Math2.ROTATION2D_PI)) : state;
        lastMoveSpeed = moveSpeed;
        controlTimer.restart();
    }

    /**
     * Drives the swerve module using voltage.
     * @param voltage The voltage to apply to the move motor.
     * @param heading The desired heading of the turn motor.
     */
    public void setVoltage(double voltage, Rotation2d heading) {
        double turnTarget = turnMotor.getPosition() + heading.rotateBy(Rotation2d.fromRadians(getHeading()).times(-1.0)).getRadians();

        moveMotor.setVoltage(voltage);
        turnMotor.setReference(turnTarget, 0.0);

        double expectedVelocity =
            moveFF.maxAchievableVelocity(
                config.getOptimalVoltage(),
                controlTimer.get() == 0 ? 0.0 : (simVelocity - lastMoveSpeed) / controlTimer.get()
            ) *
            (voltage / config.getOptimalVoltage());

        if (RobotBase.isSimulation()) {
            simDistance += simVelocity * controlTimer.get();
            simHeading = heading.getRadians();
            simVelocity = expectedVelocity;
        }

        desiredState = new SwerveModuleState(expectedVelocity, heading);
        lastMoveSpeed = RobotBase.isSimulation() ? simVelocity : getVelocity();
        controlTimer.restart();
    }

    /**
     * Returns an integer representing the number of devices with a read error.
     * Minimum of {@code 0}, maximum of {@code 3} (Move Motor, Turn Motor, Encoder).
     */
    public int readErrorCount() {
        int errors = 0;
        if (moveMotor.readError()) errors++;
        if (turnMotor.readError()) errors++;
        if (encoder.readError()) errors++;
        return errors;
    }
}
