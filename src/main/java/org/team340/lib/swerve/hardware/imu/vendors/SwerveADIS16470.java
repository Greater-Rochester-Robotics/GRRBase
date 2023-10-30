package org.team340.lib.swerve.hardware.imu.vendors;

import org.team340.lib.drivers.ADIS16470;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;

/**
 * ADIS 16470 swerve wrapper.
 */
public class SwerveADIS16470 implements SwerveIMU {

    /**
     * The IMU.
     */
    private final ADIS16470 imu;

    /**
     * Create the ADIS 16470 wrapper.
     * @param imu The ADIS 16470 to wrap.
     */
    public SwerveADIS16470(ADIS16470 imu) {
        this.imu = imu;
    }

    @Override
    public double getYaw() {
        return Math.toRadians(imu.getAngle(imu.getYawAxis()) % 360.0);
    }

    @Override
    public double getPitch() {
        return Math.toRadians(imu.getAngle(imu.getPitchAxis()) % 360.0);
    }

    @Override
    public double getRoll() {
        return Math.toRadians(imu.getAngle(imu.getRollAxis()) % 360.0);
    }

    @Override
    public void setZero(double yaw) {
        imu.resetAllAngles();
        imu.setGyroAngle(imu.getYawAxis(), yaw);
    }
}
