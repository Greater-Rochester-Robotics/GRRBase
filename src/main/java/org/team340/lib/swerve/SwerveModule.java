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
class SwerveModule {

    private final SwerveConfig config;
    private final SwerveModuleConfig moduleConfig;
    private final SwerveMotor moveMotor;
    private final SwerveMotor turnMotor;
    private final SwerveEncoder encoder;
    private final SimpleMotorFeedforward moveFFController;
    private final Timer controlTimer = new Timer();

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
    public SwerveModule(
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

        moveFFController = new SimpleMotorFeedforward(config.getMoveFF().s(), config.getMoveFF().v(), config.getMoveFF().a());
    }

    /**
     * Gets the module's label.
     */
    public String getLabel() {
        return moduleConfig.getLabel();
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

    public double getMoveDutyCycle() {
        if (RobotBase.isSimulation()) {
            return (simVelocity / config.getMaxV()) * 12.0;
        } else {
            return moveMotor.getDutyCycle();
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
     * Sets a desired state of the swerve module.
     * @param state The new state.
     * @return If the module was flipped.
     */
    public void setDesiredState(SwerveModuleState state) {
        double moveSpeed = state.speedMetersPerSecond;
        double angleDiff = state.angle.rotateBy(Rotation2d.fromRadians(getHeading()).times(-1.0)).getRadians();
        if (Math.abs(angleDiff) > (Math2.HALF_PI)) {
            if (angleDiff > 0) angleDiff -= Math.PI; else angleDiff += Math.PI;
            moveSpeed *= -1.0;
        }

        double moveFF = moveFFController.calculate(
            moveSpeed,
            controlTimer.get() == 0 ? 0.0 : (moveSpeed - lastMoveSpeed) / controlTimer.get()
        );
        double turnTarget = turnMotor.getPosition() + angleDiff;

        moveMotor.setReference(moveSpeed, moveFF);
        turnMotor.setReference(turnTarget, 0.0);

        if (RobotBase.isSimulation()) {
            simDistance += simVelocity * controlTimer.get();
            simHeading =
                Math.signum(moveSpeed) == Math.signum(state.speedMetersPerSecond)
                    ? state.angle.getRadians()
                    : state.angle.minus(Math2.ROTATION2D_PI).getRadians();
            simVelocity = moveSpeed;
        }

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

        if (RobotBase.isSimulation()) {
            simDistance += simVelocity * controlTimer.get();
            simHeading = heading.getRadians();
            simVelocity =
                moveFFController.maxAchievableVelocity(
                    config.getOptimalVoltage(),
                    controlTimer.get() == 0 ? 0.0 : (simVelocity - lastMoveSpeed) / controlTimer.get()
                ) *
                (voltage / config.getOptimalVoltage());
        }

        lastMoveSpeed = RobotBase.isSimulation() ? simVelocity : getVelocity();
        controlTimer.restart();
    }
}
