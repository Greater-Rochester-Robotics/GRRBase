package org.team340.lib.swerve.hardware.imu.vendors;

import com.ctre.phoenix6.hardware.Pigeon2;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;

/**
 * CTRE Pigeon 2 swerve wrapper.
 */
public class SwervePigeon2 implements SwerveIMU {

    /**
     * The IMU.
     */
    private final Pigeon2 imu;

    /**
     * Create the Pigeon 2 wrapper.
     * @param imu The Pigeon 2 to wrap.
     * @param config General config.
     */
    public SwervePigeon2(Pigeon2 imu, SwerveConfig config) {
        this.imu = imu;

        double hz = 1.0 / config.getPeriod();
        imu.getYaw().setUpdateFrequency(hz);
        imu.getPitch().setUpdateFrequency(hz);
        imu.getRoll().setUpdateFrequency(hz);
    }

    @Override
    public double getYaw() {
        return Math.toRadians(imu.getYaw().getValue() % 360.0);
    }

    @Override
    public double getPitch() {
        return Math.toRadians(imu.getPitch().getValue() % 360.0);
    }

    @Override
    public double getRoll() {
        return Math.toRadians(imu.getRoll().getValue() % 360.0);
    }

    @Override
    public void setZero(double yaw) {
        imu.reset();
        imu.setYaw(yaw);
    }
}
