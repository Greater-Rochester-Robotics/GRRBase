package org.team340.lib.swerve.simulation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;

/**
 * A simulated IMU.
 */
public class SwerveSimIMU implements SwerveIMU {

    private final Timer timer = new Timer();
    private double lastTime;
    private double angle;

    /**
     * Create the simulated IMU.
     */
    public SwerveSimIMU() {
        timer.start();
        lastTime = timer.get();
    }

    @Override
    public double getYaw() {
        return angle;
    }

    @Override
    public double getPitch() {
        return 0.0;
    }

    @Override
    public double getRoll() {
        return 0.0;
    }

    @Override
    public void setZero(double yaw) {
        angle = yaw;
    }

    /**
     * Updates the simulated IMU's angle based on chassis speeds.
     */
    public void updateAngle(ChassisSpeeds speeds) {
        double now = timer.get();
        angle += speeds.omegaRadiansPerSecond * (now - lastTime);
        lastTime = now;
    }
}
