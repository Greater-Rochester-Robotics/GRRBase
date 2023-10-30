package org.team340.lib.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveAbsoluteEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.Math2;

/**
 * A swerve module for {@link SwerveBase}
 */
public class SwerveModule {

    /**
     * The module's config.
     */
    private final SwerveModuleConfig moduleConfig;

    /**
     * The move motor used by the module.
     */
    private final SwerveMotor moveMotor;
    /**
     * The turn motor used by the module.
     */
    private final SwerveMotor turnMotor;
    /**
     * The absolute encoder used by the module.
     */
    private final SwerveAbsoluteEncoder absoluteEncoder;

    /**
     * Feed forward calculator for the move motor.
     */
    private final SimpleMotorFeedforward moveFF;

    /**
     * Create the swerve module.
     * @param moveMotor The module's move motor.
     * @param turnMotor The module's turn motor.
     * @param config The general swerve config.
     * @param moduleConfig The module's config.
     */
    public SwerveModule(
        SwerveMotor moveMotor,
        SwerveMotor turnMotor,
        SwerveAbsoluteEncoder absoluteEncoder,
        SwerveConfig config,
        SwerveModuleConfig moduleConfig
    ) {
        this.moduleConfig = moduleConfig;
        this.moveMotor = moveMotor;
        this.turnMotor = turnMotor;
        this.absoluteEncoder = absoluteEncoder;

        double[] moveFFConstants = config.getMoveFF();
        moveFF = new SimpleMotorFeedforward(moveFFConstants[0], moveFFConstants[1], moveFFConstants[2]);
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
        return moveMotor.getRelativePosition();
    }

    /**
     * Gets the absolute angle of the swerve module in radians.
     */
    public double getAngle() {
        return absoluteEncoder.getAbsolutePosition();
    }

    /**
     * Gets the current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromRadians(getAngle()));
    }

    /**
     * Gets the current position of the swerve module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistance(), Rotation2d.fromRadians(getAngle()));
    }

    /**
     * Sets a desired state of the swerve module.
     * @param state The new state.
     * @return If the module was flipped.
     */
    public boolean setDesiredState(SwerveModuleState state) {
        boolean flip = false;
        double moveSpeed = state.speedMetersPerSecond;
        double angleDiff = state.angle.rotateBy(Rotation2d.fromRadians(getAngle()).times(-1.0)).getRadians();
        if (Math.abs(angleDiff) > (Math2.HALF_PI)) {
            if (angleDiff > 0) angleDiff -= Math.PI; else angleDiff += Math.PI;
            moveSpeed *= -1.0;
            flip = true;
        }

        moveMotor.setReference(moveSpeed, moveFF.calculate(moveSpeed));
        turnMotor.setReference(turnMotor.getRelativePosition() + angleDiff, 0.0);

        return flip;
    }
}
