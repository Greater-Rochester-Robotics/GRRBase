package org.team340.lib.swerve.hardware.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.team340.lib.util.Math2;

/**
 * An IMU wrapper for swerve.
 */
public abstract class SwerveIMU {

    private final Timer simTimer;
    private Rotation2d simYaw = Math2.ROTATION2D_0;
    private double simYawRadiansPerSecond = 0.0;

    public SwerveIMU() {
        if (RobotBase.isSimulation()) {
            simTimer = new Timer();
        } else {
            simTimer = null;
        }
    }

    /**
     * Gets the IMU's yaw.
     */
    protected abstract Rotation2d getRealYaw();

    /**
     * Gets the IMU's pitch.
     */
    protected abstract Rotation2d getRealPitch();

    /**
     * Gets the IMU's roll.
     */
    protected abstract Rotation2d getRealRoll();

    /**
     * Zero the pitch and roll of the IMU and set the yaw to a specified angle.
     * @param yaw The yaw to zero to.
     */
    protected abstract void setRealZero(Rotation2d yaw);

    /**
     * Gets the IMU's yaw.
     */
    public Rotation2d getYaw() {
        if (RobotBase.isSimulation()) {
            return simYaw;
        } else {
            return getRealYaw();
        }
    }

    /**
     * Gets the IMU's pitch.
     */
    public Rotation2d getPitch() {
        if (RobotBase.isSimulation()) {
            return Math2.ROTATION2D_0;
        } else {
            return getRealPitch();
        }
    }

    /**
     * Gets the IMU's roll.
     */
    public Rotation2d getRoll() {
        if (RobotBase.isSimulation()) {
            return Math2.ROTATION2D_0;
        } else {
            return getRealRoll();
        }
    }

    /**
     * Zero the pitch and roll of the IMU and set the yaw to a specified angle in radians.
     * @param yaw The yaw to zero to in radians.
     */
    public void setZero(Rotation2d yaw) {
        simYaw = yaw;
    }

    /**
     * Updates the simulated IMU.
     * Has no effect when ran on a real robot.
     * @param speeds The latest commanded chassis speeds.
     */
    public void updateSim(ChassisSpeeds speeds) {
        if (!RobotBase.isSimulation()) return;
        simYaw = simYaw.plus(Rotation2d.fromRadians(simYawRadiansPerSecond * simTimer.get()));
        simYawRadiansPerSecond = speeds.omegaRadiansPerSecond;
        simTimer.restart();
    }
}
