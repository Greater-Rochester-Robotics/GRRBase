package org.team340.lib.swerve.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * An IMU wrapper for swerve.
 */
public interface SwerveIMU {
    /**
     * Supported IMUs.
     */
    public static enum Type {
        ADIS16470,
        PIGEON2,
    }

    /**
     * Gets the IMU's absolute yaw.
     */
    public Rotation2d getYaw();

    /**
     * Gets the IMU's absolute pitch.
     */
    public Rotation2d getPitch();

    /**
     * Gets the IMU's absolute roll.
     */
    public Rotation2d getRoll();

    /**
     * Zero the pitch and roll of the IMU and set the yaw to a specified angle.
     * @param yaw The yaw to zero.
     */
    public void setZero(Rotation2d yaw);

    /**
     * If the device has encountered an error while reading inputs.
     */
    public default boolean readError() {
        return false;
    }
}
