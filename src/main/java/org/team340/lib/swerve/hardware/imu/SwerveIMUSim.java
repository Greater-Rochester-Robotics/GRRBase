package org.team340.lib.swerve.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.team340.lib.util.Math2;

public class SwerveIMUSim implements SwerveIMU {

    private final Timer timer = new Timer();

    private Rotation2d yaw = Math2.ROTATION2D_0;
    private double yawRadiansPerSecond = 0.0;

    /**
     * Updates the simulated IMU using the robot's current chassis speeds.
     * @param newSpeeds The new robot speeds.
     */
    public void updateSim(ChassisSpeeds newSpeeds) {
        yaw = yaw.plus(Rotation2d.fromRadians(yawRadiansPerSecond * timer.get()));
        yawRadiansPerSecond = newSpeeds.omegaRadiansPerSecond;
        timer.restart();
    }

    @Override
    public Rotation2d getYaw() {
        return yaw;
    }

    @Override
    public Rotation2d getPitch() {
        return Math2.ROTATION2D_0;
    }

    @Override
    public Rotation2d getRoll() {
        return Math2.ROTATION2D_0;
    }

    @Override
    public void setZero(Rotation2d yaw) {
        this.yaw = yaw;
    }
}
