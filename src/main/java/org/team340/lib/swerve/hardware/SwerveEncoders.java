package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.function.BiFunction;
import org.team340.lib.logging.CANcoderLogger;
import org.team340.lib.logging.SparkAbsoluteEncoderLogger;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.SwerveMotors.SwerveMotor;
import org.team340.lib.util.Mutable;
import org.team340.lib.util.ctre.PhoenixUtil;
import org.team340.lib.util.rev.SparkAbsoluteEncoderConfig;
import org.team340.lib.util.rev.SparkFlexConfig;
import org.team340.lib.util.rev.SparkMaxConfig;

/**
 * Contains implementations for absolute encoders to be used with the {@link SwerveAPI}.
 */
public final class SwerveEncoders {

    private SwerveEncoders() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * A swerve module's absolute encoder.
     * All units are in rotations.
     */
    public abstract static class SwerveEncoder implements SwerveBaseHardware {

        /**
         * {@code (config, turnMotor) -> SwerveEncoder}
         */
        @FunctionalInterface
        public static interface Ctor extends BiFunction<SwerveConfig, SwerveMotor, SwerveEncoder> {}

        /**
         * Constructs a swerve encoder. Wraps to support simulation if applicable.
         * @param ctor The encoder's constructor.
         * @param config The general swerve API configuration.
         * @param turnMotor The turn motor associated with the encoder's module.
         */
        public static SwerveEncoder construct(Ctor ctor, SwerveConfig config, SwerveMotor turnMotor) {
            SwerveEncoder encoder = ctor.apply(config, turnMotor);
            if (RobotBase.isSimulation()) encoder = simulate(encoder, config, turnMotor);
            return encoder;
        }

        /**
         * Gets the encoder's position in rotations.
         */
        public abstract double getPosition();

        /**
         * Some motor controllers can be configured to use external encoders
         * as a feedback device for closed-loop control. This method returns
         * {@code true} if the encoder's corresponding turn motor has been
         * configured to accept an absolute position when setting its closed
         * loop position target, as well as return an absolute position via
         * its {@code getPosition()} method.
         */
        public boolean hookStatus() {
            return false;
        }
    }

    /**
     * Configures a Spark Max attached duty cycle absolute encoder.
     * @param offset Offset of the magnet in rotations.
     * @param inverted If the encoder is inverted.
     */
    public static SwerveEncoder.Ctor sparkMaxEncoder(double offset, boolean inverted) {
        return (config, turnMotor) -> {
            if (!(turnMotor.getAPI() instanceof CANSparkMax)) {
                throw new UnsupportedOperationException("Turn motor is not a Spark Max");
            }

            var deviceLogger = new SparkAbsoluteEncoderLogger();
            CANSparkMax sparkMax = (CANSparkMax) turnMotor.getAPI();
            SparkAbsoluteEncoder encoder = sparkMax.getAbsoluteEncoder();

            new SparkMaxConfig()
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S5, (int) (config.odometryPeriod * 1000.0))
                .setPeriodicFramePeriod(
                    SparkMaxConfig.Frame.S6,
                    (int) (SwerveBaseHardware.TELEMETRY_CAN_PERIOD * 1000.0)
                )
                .apply(sparkMax);

            new SparkAbsoluteEncoderConfig()
                .setPositionConversionFactor(1.0)
                .setVelocityConversionFactor(1.0 / 60.0)
                .setInverted(inverted)
                .setZeroOffset(offset)
                .apply(sparkMax, encoder);

            return new SwerveEncoder() {
                @Override
                public double getPosition() {
                    return encoder.getPosition();
                }

                @Override
                public Object getAPI() {
                    return encoder;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, encoder, errorHandler);
                }

                @Override
                public void close() {}
            };
        };
    }

    /**
     * Configures a Spark Flex attached duty cycle absolute encoder.
     * @param offset Offset of the magnet in rotations.
     * @param inverted If the encoder is inverted.
     */
    public static SwerveEncoder.Ctor sparkFlexEncoder(double offset, boolean inverted) {
        return (config, turnMotor) -> {
            if (!(turnMotor.getAPI() instanceof CANSparkFlex)) {
                throw new UnsupportedOperationException("Turn motor is not a Spark Flex");
            }

            var deviceLogger = new SparkAbsoluteEncoderLogger();
            CANSparkFlex sparkFlex = (CANSparkFlex) turnMotor.getAPI();
            SparkAbsoluteEncoder encoder = sparkFlex.getAbsoluteEncoder();

            new SparkFlexConfig()
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S5, (int) (config.odometryPeriod * 1000.0))
                .setPeriodicFramePeriod(
                    SparkFlexConfig.Frame.S6,
                    (int) (SwerveBaseHardware.TELEMETRY_CAN_PERIOD * 1000.0)
                )
                .apply(sparkFlex);

            new SparkAbsoluteEncoderConfig()
                .setPositionConversionFactor(1.0)
                .setVelocityConversionFactor(1.0 / 60.0)
                .setInverted(inverted)
                .setZeroOffset(offset)
                .apply(sparkFlex, encoder);

            return new SwerveEncoder() {
                @Override
                public double getPosition() {
                    return encoder.getPosition();
                }

                @Override
                public Object getAPI() {
                    return encoder;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, encoder, errorHandler);
                }

                @Override
                public void close() {}
            };
        };
    }

    /**
     * Configures a {@link CANcoder}.
     * @param id CAN ID of the device, as configured in Phoenix Tuner.
     * @param offset Offset of the magnet in rotations.
     * @param inverted If the encoder is inverted.
     */
    public static SwerveEncoder.Ctor canCoder(int id, double offset, boolean inverted) {
        return (config, turnMotor) -> {
            var deviceLogger = new CANcoderLogger();
            CANcoder canCoder = new CANcoder(id, config.phoenixCanBus);
            Mutable<Boolean> hookStatus = new Mutable<>(false);

            StatusSignal<Double> position = canCoder.getPosition().clone();
            StatusSignal<Double> velocity = canCoder.getVelocity().clone();

            var canCoderConfig = new CANcoderConfiguration();
            canCoderConfig.MagnetSensor.MagnetOffset = offset;
            canCoderConfig.MagnetSensor.SensorDirection = inverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;

            PhoenixUtil.run(canCoder, "Apply CANcoderConfiguration", () ->
                canCoder.getConfigurator().apply(canCoderConfig)
            );

            BaseStatusSignal.setUpdateFrequencyForAll(1.0 / config.odometryPeriod, position, velocity);
            canCoder.optimizeBusUtilization(1.0 / SwerveBaseHardware.TELEMETRY_CAN_PERIOD, 0.05);

            if (turnMotor.getAPI() instanceof TalonFX) {
                TalonFX talonFX = (TalonFX) turnMotor.getAPI();

                var feedbackConfig = new FeedbackConfigs();
                feedbackConfig.FeedbackRemoteSensorID = id;
                feedbackConfig.RotorToSensorRatio = config.turnGearRatio;
                feedbackConfig.FeedbackSensorSource = config.phoenixPro
                    ? FeedbackSensorSourceValue.FusedCANcoder
                    : FeedbackSensorSourceValue.RemoteCANcoder;

                var closedLoopConfig = new ClosedLoopGeneralConfigs();
                closedLoopConfig.ContinuousWrap = true;

                PhoenixUtil.run(talonFX, "Apply FeedbackConfigs", () -> talonFX.getConfigurator().apply(feedbackConfig)
                );
                PhoenixUtil.run(talonFX, "Apply ClosedLoopGeneralConfigs", () ->
                    talonFX.getConfigurator().apply(closedLoopConfig)
                );

                hookStatus.value = true;
            }

            return new SwerveEncoder() {
                @Override
                public double getPosition() {
                    return BaseStatusSignal.getLatencyCompensatedValue(position, velocity);
                }

                @Override
                public boolean hookStatus() {
                    return hookStatus.get();
                }

                @Override
                public Object getAPI() {
                    return canCoder;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, canCoder, errorHandler);
                }

                @Override
                public List<BaseStatusSignal> getSignals() {
                    return List.of(position, velocity);
                }

                @Override
                public void close() {
                    canCoder.close();
                }
            };
        };
    }

    /**
     * Rudimentary encoder simulation wrapper. Follows the position of the turn motor.
     * @param encoder The encoder to wrap.
     * @param config The general swerve API configuration.
     * @param turnMotor The turn motor associated with the encoder's module.
     */
    private static SwerveEncoder simulate(SwerveEncoder encoder, SwerveConfig config, SwerveMotor turnMotor) {
        return new SwerveEncoder() {
            @Override
            public double getPosition() {
                return turnMotor.getPosition() / (hookStatus() ? 1.0 : config.turnGearRatio);
            }

            @Override
            public boolean hookStatus() {
                return encoder.hookStatus();
            }

            @Override
            public Object getAPI() {
                return encoder;
            }

            @Override
            public void log(DataLogger logger, ErrorHandler errorHandler) {
                encoder.log(logger, errorHandler);
                var simLogger = logger.getSubLogger(".sim");
                simLogger.log("position", getPosition());
                simLogger.log("hookStatus", hookStatus());
            }

            @Override
            public List<BaseStatusSignal> getSignals() {
                return encoder.getSignals();
            }

            @Override
            public boolean readError() {
                return encoder.readError();
            }

            @Override
            public void close() {
                try {
                    encoder.close();
                } catch (Exception e) {}
            }
        };
    }
}
