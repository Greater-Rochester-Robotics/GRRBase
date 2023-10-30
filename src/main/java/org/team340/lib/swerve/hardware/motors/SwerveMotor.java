package org.team340.lib.swerve.hardware.motors;

/**
 * A motor for swerve, can be a move or turn motor.
 * Bound to {@link SwerveModule}s.
 */
public interface SwerveMotor {
    /**
     * If the motor is a move motor.
     */
    public abstract boolean isMoveMotor();

    /**
     * Gets the motor's velocity.
     */
    public abstract double getVelocity();

    /**
     * Gets the motor's relative position.
     */
    public abstract double getRelativePosition();

    /**
     * Sets the motor's closed loop target.
     * If the motor is a move motor, the target is in meters/second.
     * If the motor is a turn motor, the target is an unclamped position in radians.
     * @param target The target.
     * @param ff Arbitrary feed forward. Only applied to move motors.
     */
    public abstract void setReference(double target, double ff);
}
