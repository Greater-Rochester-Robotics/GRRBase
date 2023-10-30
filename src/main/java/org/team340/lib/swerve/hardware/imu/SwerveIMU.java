package org.team340.lib.swerve.hardware.imu;

/**
 * An IMU wrapper for swerve.
 */
public interface SwerveIMU {
    /**
     * Gets the IMU's absolute yaw in radians.
     * Clamped from {@code -2 PI} to {@code 2 PI}.
     */
    public double getYaw();

    /**
     * Gets the IMU's absolute pitch in radians.
     * Clamped from {@code -2 PI} to {@code 2 PI}.
     */
    public double getPitch();

    /**
     * Gets the IMU's absolute roll in radians.
     * Clamped from {@code -2 PI} to {@code 2 PI}.
     */
    public double getRoll();

    /**
     * Zero the pitch and roll of the IMU and set the yaw to a specified angle in radians.
     * @param yaw The yaw to zero to in radians.
     */
    public void setZero(double yaw);
}
