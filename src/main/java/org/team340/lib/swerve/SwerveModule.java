package org.team340.lib.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.Math2;

/**
 * A swerve module for {@link SwerveBase}
 */
public class SwerveModule {

    private final SwerveModuleConfig moduleConfig;
    private final SwerveMotor moveMotor;
    private final SwerveMotor turnMotor;
    private final SwerveEncoder encoder;
    private final SimpleMotorFeedforward moveFFController;

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
        return moveMotor.getVelocity();
    }

    /**
     * Gets the distance traveled by the swerve module in meters.
     */
    public double getDistance() {
        return moveMotor.getPosition();
    }

    /**
     * Gets the absolute angle of the swerve module in radians.
     */
    public double getAbsoluteAngle() {
        return encoder.getPosition();
    }

    /**
     * Gets the current state of the swerve module.
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromRadians(getAbsoluteAngle()));
    }

    /**
     * Gets the current position of the swerve module.
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromRadians(getAbsoluteAngle()));
    }

    /**
     * Sets a desired state of the swerve module.
     * @param state The new state.
     * @return If the module was flipped.
     */
    public void setDesiredState(SwerveModuleState state) {
        double moveSpeed = state.speedMetersPerSecond;
        double angleDiff = state.angle.rotateBy(Rotation2d.fromRadians(getAbsoluteAngle()).times(-1.0)).getRadians();
        if (Math.abs(angleDiff) > (Math2.HALF_PI)) {
            if (angleDiff > 0) angleDiff -= Math.PI; else angleDiff += Math.PI;
            moveSpeed *= -1.0;
        }

        double turnTarget = turnMotor.getPosition() + angleDiff;

        moveMotor.setReference(moveSpeed, moveFFController.calculate(moveSpeed));
        turnMotor.setReference(turnTarget, 0.0);

        if (RobotBase.isSimulation()) encoder.setSimPosition(turnTarget);
    }
}
