package org.team340.lib.swerve.hardware.imu.vendors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;

/**
 * CTRE Pigeon 2 swerve wrapper.
 */
public class SwervePigeon2 extends SwerveIMU {

    private final Pigeon2 pigeon2;
    private final StatusSignal<Double> yawSignal;
    private final StatusSignal<Double> pitchSignal;
    private final StatusSignal<Double> rollSignal;

    /**
     * Create the Pigeon 2 wrapper.
     * @param pigeon2 The Pigeon 2 to wrap.
     * @param config General config.
     */
    public SwervePigeon2(Pigeon2 pigeon2, SwerveConfig config) {
        this.pigeon2 = pigeon2;

        yawSignal = pigeon2.getYaw();
        pitchSignal = pigeon2.getPitch();
        rollSignal = pigeon2.getRoll();

        double hz = 1.0 / config.getPeriod();
        yawSignal.setUpdateFrequency(hz);
        pitchSignal.setUpdateFrequency(hz);
        rollSignal.setUpdateFrequency(hz);
    }

    @Override
    protected Rotation2d getRealYaw() {
        return Rotation2d.fromRadians(yawSignal.refresh().getValue());
    }

    @Override
    protected Rotation2d getRealPitch() {
        return Rotation2d.fromRadians(pitchSignal.refresh().getValue());
    }

    @Override
    protected Rotation2d getRealRoll() {
        return Rotation2d.fromRadians(rollSignal.refresh().getValue());
    }

    @Override
    protected void setRealZero(Rotation2d yaw) {
        pigeon2.reset();
        pigeon2.setYaw(yaw.getDegrees());
    }
}
