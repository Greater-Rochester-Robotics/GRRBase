package org.team340.lib.swerve.hardware.imu.vendors;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;

/**
 * ADIS 16470 swerve wrapper.
 */
public class SwerveADIS16470 implements SwerveIMU {

    private final ADIS16470_IMU adis16470;

    /**
     * Create the ADIS 16470 wrapper.
     * @param adis16470 The ADIS 16470 to wrap.
     */
    public SwerveADIS16470(ADIS16470_IMU adis16470) {
        this.adis16470 = adis16470;
    }

    @Override
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getYawAxis()));
    }

    @Override
    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getPitchAxis()));
    }

    @Override
    public Rotation2d getRoll() {
        return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getRollAxis()));
    }

    @Override
    public void setZero(Rotation2d yaw) {
        adis16470.setGyroAngle(adis16470.getYawAxis(), yaw.getDegrees());
        adis16470.setGyroAngle(adis16470.getPitchAxis(), 0.0);
        adis16470.setGyroAngle(adis16470.getRollAxis(), 0.0);
    }
}
