package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI;
import java.util.List;
import java.util.function.Function;
import org.team340.lib.logging.ADIS16470Logger;
import org.team340.lib.logging.Pigeon2Logger;
import org.team340.lib.swerve.config.SwerveConfig;

public final class SwerveIMUs {

    private SwerveIMUs() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * A swerve IMU.
     */
    public abstract static class SwerveIMU implements SwerveBaseHardware {

        /**
         * {@code (config) -> SwerveIMU}
         */
        @FunctionalInterface
        public static interface Ctor extends Function<SwerveConfig, SwerveIMU> {}

        /**
         * Gets the IMU's absolute yaw.
         */
        public abstract Rotation2d getYaw();

        /**
         * Gets the IMU's absolute pitch.
         */
        public abstract Rotation2d getPitch();

        /**
         * Gets the IMU's absolute roll.
         */
        public abstract Rotation2d getRoll();

        /**
         * Zero the pitch and roll of the IMU and set the yaw to a specified angle.
         * @param yaw The yaw to zero.
         */
        public abstract void setZero(Rotation2d yaw);
    }

    /**
     * Configures an {@link ADIS16470_IMU ADIS16470 IMU}.
     * @param yawAxis The axis to use for yaw.
     * @param pitchAxis The axis to use for pitch.
     * @param rollAxis The axis to use for roll.
     * @param port The SPI port used.
     * @param calibrationTime The time frame to calibrate for.
     */
    public static SwerveIMU.Ctor adis16470(
        IMUAxis yawAxis,
        IMUAxis pitchAxis,
        IMUAxis rollAxis,
        SPI.Port port,
        CalibrationTime calibrationTime
    ) {
        return config -> {
            var deviceLogger = new ADIS16470Logger();
            ADIS16470_IMU adis16470 = new ADIS16470_IMU(yawAxis, pitchAxis, rollAxis, port, calibrationTime);

            return new SwerveIMU() {
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

                @Override
                public Object getAPI() {
                    return adis16470;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, adis16470, errorHandler);
                }

                @Override
                public void close() {
                    adis16470.close();
                }
            };
        };
    }

    /**
     * Configures a {@link Pigeon2}.
     * @param id CAN ID of the device, as configured in Phoenix Tuner.
     */
    public static SwerveIMU.Ctor pigeon2(int id) {
        return config -> {
            var deviceLogger = new Pigeon2Logger();
            Pigeon2 pigeon2 = new Pigeon2(id, config.getPhoenixCanBus());

            StatusSignal<Double> yaw = pigeon2.getYaw().clone();
            StatusSignal<Double> pitch = pigeon2.getPitch().clone();
            StatusSignal<Double> roll = pigeon2.getRoll().clone();
            StatusSignal<Double> yawVelocity = pigeon2.getAngularVelocityZWorld().clone();
            StatusSignal<Double> pitchVelocity = pigeon2.getAngularVelocityXWorld().clone();
            StatusSignal<Double> rollVelocity = pigeon2.getAngularVelocityYWorld().clone();

            BaseStatusSignal.setUpdateFrequencyForAll(
                1.0 / config.getOdometryPeriod(),
                yaw,
                pitch,
                roll,
                yawVelocity,
                pitchVelocity,
                rollVelocity
            );
            pigeon2.optimizeBusUtilization(1.0 / SwerveBaseHardware.TELEMETRY_CAN_PERIOD, 0.05);

            return new SwerveIMU() {
                @Override
                public Rotation2d getYaw() {
                    return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yaw, yawVelocity));
                }

                @Override
                public Rotation2d getPitch() {
                    return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(pitch, pitchVelocity));
                }

                @Override
                public Rotation2d getRoll() {
                    return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(roll, rollVelocity));
                }

                @Override
                public void setZero(Rotation2d yaw) {
                    pigeon2.reset();
                    pigeon2.setYaw(yaw.getDegrees());
                }

                @Override
                public Object getAPI() {
                    return pigeon2;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, pigeon2, errorHandler);
                }

                @Override
                public List<BaseStatusSignal> getSignals() {
                    return List.of(yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
                }

                @Override
                public void close() {
                    pigeon2.close();
                }
            };
        };
    }
}
