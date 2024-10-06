package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.team340.lib.logging.ADIS16470Logger;
import org.team340.lib.logging.Pigeon2Logger;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.config.SwerveConfig;

/**
 * Contains implementations for IMUs to be used with the {@link SwerveAPI}.
 */
public final class SwerveIMUs {

    private SwerveIMUs() {
        throw new AssertionError("This is a utility class!");
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
         * Constructs a swerve IMU. Wraps to support simulation if applicable.
         * @param ctor The IMU's constructor.
         * @param config The general swerve API configuration.
         * @param speedsSupplier A supplier for the robot's current chassis speeds. Used only for simulation.
         */
        public static SwerveIMU construct(Ctor ctor, SwerveConfig config, Supplier<ChassisSpeeds> speedsSupplier) {
            SwerveIMU imu = ctor.apply(config);
            if (RobotBase.isSimulation()) imu = simulate(imu, config, speedsSupplier);
            return imu;
        }

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
            Pigeon2 pigeon2 = new Pigeon2(id, config.phoenixCanBus);

            StatusSignal<Double> yaw = pigeon2.getYaw().clone();
            StatusSignal<Double> pitch = pigeon2.getPitch();
            StatusSignal<Double> roll = pigeon2.getRoll();
            StatusSignal<Double> yawVelocity = pigeon2.getAngularVelocityZWorld().clone();
            StatusSignal<Double> pitchVelocity = pigeon2.getAngularVelocityXWorld();
            StatusSignal<Double> rollVelocity = pigeon2.getAngularVelocityYWorld();

            BaseStatusSignal.setUpdateFrequencyForAll(1.0 / config.odometryPeriod, yaw, yawVelocity);
            BaseStatusSignal.setUpdateFrequencyForAll(1.0 / config.period, pitch, roll, pitchVelocity, rollVelocity);
            pigeon2.optimizeBusUtilization(1.0 / SwerveBaseHardware.TELEMETRY_CAN_PERIOD, 0.05);

            return new SwerveIMU() {
                @Override
                public Rotation2d getYaw() {
                    return Rotation2d.fromDegrees(BaseStatusSignal.getLatencyCompensatedValue(yaw, yawVelocity));
                }

                @Override
                public Rotation2d getPitch() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValue(pitch.refresh(), pitchVelocity.refresh())
                    );
                }

                @Override
                public Rotation2d getRoll() {
                    return Rotation2d.fromDegrees(
                        BaseStatusSignal.getLatencyCompensatedValue(roll.refresh(), rollVelocity.refresh())
                    );
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
                    return List.of(yaw, yawVelocity);
                }

                @Override
                public void close() {
                    pigeon2.close();
                }
            };
        };
    }

    /**
     * Rudimentary IMU simulation wrapper. Calculates yaw based on the robot's angular velocity.
     * @param imu The IMU to wrap.
     * @param config The general swerve API configuration.
     * @param speedsSupplier A supplier that returns the robot's current chassis speeds.
     */
    private static SwerveIMU simulate(SwerveIMU imu, SwerveConfig config, Supplier<ChassisSpeeds> speedsSupplier) {
        return new SwerveIMU() {
            private Rotation2d yaw = Rotation2d.kZero;

            @Override
            public Rotation2d getYaw() {
                yaw = yaw.plus(Rotation2d.fromRadians(speedsSupplier.get().omegaRadiansPerSecond * config.period));
                return yaw;
            }

            @Override
            public Rotation2d getPitch() {
                return Rotation2d.kZero;
            }

            @Override
            public Rotation2d getRoll() {
                return Rotation2d.kZero;
            }

            @Override
            public Object getAPI() {
                return imu;
            }

            @Override
            public void log(DataLogger logger, ErrorHandler errorHandler) {
                imu.log(logger, errorHandler);
                var simLogger = logger.getSubLogger(".sim");
                simLogger.log("yaw", getYaw(), Rotation2d.struct);
                simLogger.log("pitch", getPitch(), Rotation2d.struct);
                simLogger.log("roll", getRoll(), Rotation2d.struct);
            }

            @Override
            public List<BaseStatusSignal> getSignals() {
                return imu.getSignals();
            }

            @Override
            public boolean readError() {
                return imu.readError();
            }

            @Override
            public void close() {
                try {
                    imu.close();
                } catch (Exception e) {}
            }
        };
    }
}
