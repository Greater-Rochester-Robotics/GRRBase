package org.team340.lib.swerve.hardware.motors;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

/**
 * A motor for swerve, can be a move or turn motor.
 * Bound to {@link SwerveModule}s.
 */
public interface SwerveMotor {
    /**
     * Supported motors.
     */
    public static enum Type {
        SPARK_MAX_BRUSHED,
        SPARK_MAX_BRUSHLESS,
        SPARK_FLEX_BRUSHED,
        SPARK_FLEX_BRUSHLESS,
        TALONFX,
    }

    /**
     * Gets the motor's velocity.
     */
    public abstract double getVelocity();

    /**
     * Gets the motor's relative position.
     */
    public abstract double getPosition();

    /**
     * Gets the motor's applied duty cycle.
     */
    public abstract double getDutyCycle();

    /**
     * Sets the motor's closed loop target.
     * If the motor is a move motor, the target is in meters/second.
     * If the motor is a turn motor, the target is an unclamped position in radians.
     * @param target The target.
     * @param ff Arbitrary feed forward. Only applied to move motors.
     */
    public abstract void setReference(double target, double ff);

    /**
     * Sets the motor's output voltage.
     * @param voltage The voltage to apply.
     */
    public abstract void setVoltage(double voltage);
}
