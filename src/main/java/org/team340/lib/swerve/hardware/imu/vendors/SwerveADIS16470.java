package org.team340.lib.swerve.hardware.imu.vendors;

import edu.wpi.first.math.geometry.Rotation2d;
import org.team340.lib.drivers.imu.ADIS16470;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;

/**
 * ADIS 16470 swerve wrapper.
 */
public class SwerveADIS16470 extends SwerveIMU {

    private final ADIS16470 adis16470;

    /**
     * Create the ADIS 16470 wrapper.
     * @param adis16470 The ADIS 16470 to wrap.
     */
    public SwerveADIS16470(ADIS16470 adis16470) {
        this.adis16470 = adis16470;
    }

    @Override
    protected Rotation2d getRealYaw() {
        return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getYawAxis()));
    }

    @Override
    protected Rotation2d getRealPitch() {
        return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getPitchAxis()));
    }

    @Override
    protected Rotation2d getRealRoll() {
        return Rotation2d.fromDegrees(adis16470.getAngle(adis16470.getRollAxis()));
    }

    @Override
    protected void setRealZero(Rotation2d yaw) {
        adis16470.resetAllAngles();
        adis16470.setGyroAngle(adis16470.getYawAxis(), yaw.getDegrees());
    }
}
