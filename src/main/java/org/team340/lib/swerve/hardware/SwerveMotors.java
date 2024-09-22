package org.team340.lib.swerve.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import java.util.List;
import java.util.function.BiFunction;
import org.team340.lib.logging.SparkFlexLogger;
import org.team340.lib.logging.SparkMaxLogger;
import org.team340.lib.logging.TalonFXLogger;
import org.team340.lib.rev.RelativeEncoderConfig;
import org.team340.lib.rev.SparkFlexConfig;
import org.team340.lib.rev.SparkMaxConfig;
import org.team340.lib.rev.SparkPIDControllerConfig;
import org.team340.lib.swerve.config.SwerveConfig;

public final class SwerveMotors {

    private SwerveMotors() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * A swerve motor.
     * All units are in rotations.
     */
    public abstract static class SwerveMotor implements SwerveBaseHardware {

        /**
         * {@code (config, isMoveMotor) -> SwerveMotor}
         */
        @FunctionalInterface
        public static interface Ctor extends BiFunction<SwerveConfig, Boolean, SwerveMotor> {}

        /**
         * Gets the motor's position in rotations.
         */
        public abstract double getPosition();

        /**
         * Sets the motor's closed-loop position target.
         * @param position The target position in rotations.
         */
        public abstract void setPosition(double position);

        /**
         * Gets the motor's velocity in rotations/second.
         */
        public abstract double getVelocity();

        /**
         * Sets the motor's closed-loop velocity target.
         * @param velocity The target velocity in rotations/second.
         */
        public abstract void setVelocity(double velocity);

        /**
         * Sets the motor's output voltage.
         * @param voltage The voltage to apply.
         */
        public abstract void setVoltage(double voltage);
    }

    /**
     * Configures a {@link CANSparkMax Spark Max}.
     * @param id CAN ID of the device, as configured in the REV Hardware Client.
     * @param type The motor type connected to the controller.
     * @param inverted If the motor is inverted.
     */
    public static SwerveMotor.Ctor sparkMax(int id, MotorType type, boolean inverted) {
        return (config, isMoveMotor) -> {
            var deviceLogger = new SparkMaxLogger();
            CANSparkMax sparkMax = new CANSparkMax(id, type);
            RelativeEncoder relativeEncoder = sparkMax.getEncoder();
            SparkPIDController pidController = sparkMax.getPIDController();
            int PID_SLOT = 0;

            double[] pidGains = isMoveMotor ? config.getMovePID() : config.getTurnPID();
            double[] ffGains = isMoveMotor ? config.getMoveFF() : new double[] { 0.0, 0.0, 0.0 };

            new SparkMaxConfig()
                .clearFaults()
                .enableVoltageCompensation(config.getVoltage())
                .setSmartCurrentLimit((int) (isMoveMotor ? config.getMoveCurrentLimit() : config.getTurnCurrentLimit()))
                .setIdleMode((isMoveMotor ? config.getMoveBrakeMode() : config.getTurnBrakeMode()) ? IdleMode.kBrake : IdleMode.kCoast)
                .setInverted(inverted)
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S0, (int) (config.getPeriod() * 1000.0))
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S1, (int) (config.getOdometryPeriod() * 1000.0))
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S2, (int) (config.getOdometryPeriod() * 1000.0))
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S3, 10000)
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S4, 10000)
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S5, 10000)
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S6, 10000)
                .setPeriodicFramePeriod(SparkMaxConfig.Frame.S7, 10000)
                .apply(sparkMax);

            new SparkPIDControllerConfig()
                .setFeedbackDevice(relativeEncoder)
                .setPID(pidGains[0], pidGains[1], pidGains[2], PID_SLOT)
                .setIZone(pidGains[3], PID_SLOT)
                .apply(sparkMax, pidController);

            new RelativeEncoderConfig()
                .setPositionConversionFactor(1.0)
                .setVelocityConversionFactor(1.0 / 60.0)
                .apply(sparkMax, relativeEncoder);

            return new SwerveMotor() {
                @Override
                public double getPosition() {
                    return relativeEncoder.getPosition();
                }

                @Override
                public void setPosition(double position) {
                    pidController.setReference(position, CANSparkBase.ControlType.kPosition, PID_SLOT, 0.0, ArbFFUnits.kVoltage);
                }

                @Override
                public double getVelocity() {
                    return relativeEncoder.getVelocity();
                }

                @Override
                public void setVelocity(double velocity) {
                    pidController.setReference(
                        velocity,
                        CANSparkBase.ControlType.kVelocity,
                        PID_SLOT,
                        ffGains[0] * Math.signum(velocity) + ffGains[1] * velocity,
                        ArbFFUnits.kVoltage
                    );
                }

                @Override
                public void setVoltage(double voltage) {
                    sparkMax.setVoltage(voltage);
                }

                @Override
                public Object getAPI() {
                    return sparkMax;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, sparkMax, errorHandler);
                }

                @Override
                public boolean readError() {
                    return !sparkMax.getLastError().equals(REVLibError.kOk);
                }

                @Override
                public void close() {
                    sparkMax.close();
                }
            };
        };
    }

    /**
     * Configures a {@link CANSparkFlex Spark Flex}.
     * @param id CAN ID of the device, as configured in the REV Hardware Client.
     * @param type The motor type connected to the controller.
     * @param inverted If the motor is inverted.
     */
    public static SwerveMotor.Ctor sparkFlex(int id, MotorType type, boolean inverted) {
        return (config, isMoveMotor) -> {
            var deviceLogger = new SparkFlexLogger();
            CANSparkFlex sparkFlex = new CANSparkFlex(id, type);
            RelativeEncoder relativeEncoder = sparkFlex.getEncoder();
            SparkPIDController pidController = sparkFlex.getPIDController();
            int PID_SLOT = 0;

            double[] pidGains = isMoveMotor ? config.getMovePID() : config.getTurnPID();
            double[] ffGains = isMoveMotor ? config.getMoveFF() : new double[] { 0.0, 0.0, 0.0 };

            new SparkFlexConfig()
                .clearFaults()
                .enableVoltageCompensation(config.getVoltage())
                .setSmartCurrentLimit((int) (isMoveMotor ? config.getMoveCurrentLimit() : config.getTurnCurrentLimit()))
                .setIdleMode((isMoveMotor ? config.getMoveBrakeMode() : config.getTurnBrakeMode()) ? IdleMode.kBrake : IdleMode.kCoast)
                .setInverted(inverted)
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S0, (int) (config.getPeriod() * 1000.0))
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S1, (int) (config.getOdometryPeriod() * 1000.0))
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S2, (int) (config.getOdometryPeriod() * 1000.0))
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S3, 10000)
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S4, 10000)
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S5, 10000)
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S6, 10000)
                .setPeriodicFramePeriod(SparkFlexConfig.Frame.S7, 10000)
                .apply(sparkFlex);

            new SparkPIDControllerConfig()
                .setFeedbackDevice(relativeEncoder)
                .setPID(pidGains[0], pidGains[1], pidGains[2], PID_SLOT)
                .setIZone(pidGains[3], PID_SLOT)
                .apply(sparkFlex, pidController);

            new RelativeEncoderConfig()
                .setPositionConversionFactor(1.0)
                .setVelocityConversionFactor(1.0 / 60.0)
                .apply(sparkFlex, relativeEncoder);

            return new SwerveMotor() {
                @Override
                public double getPosition() {
                    return relativeEncoder.getPosition();
                }

                @Override
                public void setPosition(double position) {
                    pidController.setReference(position, CANSparkBase.ControlType.kPosition, PID_SLOT, 0.0, ArbFFUnits.kVoltage);
                }

                @Override
                public double getVelocity() {
                    return relativeEncoder.getVelocity();
                }

                @Override
                public void setVelocity(double velocity) {
                    pidController.setReference(
                        velocity,
                        CANSparkBase.ControlType.kVelocity,
                        PID_SLOT,
                        ffGains[0] * Math.signum(velocity) + ffGains[1] * velocity,
                        ArbFFUnits.kVoltage
                    );
                }

                @Override
                public void setVoltage(double voltage) {
                    sparkFlex.setVoltage(voltage);
                }

                @Override
                public Object getAPI() {
                    return sparkFlex;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, sparkFlex, errorHandler);
                }

                @Override
                public boolean readError() {
                    return !sparkFlex.getLastError().equals(REVLibError.kOk);
                }

                @Override
                public void close() {
                    sparkFlex.close();
                }
            };
        };
    }

    /**
     * Configures a {@link TalonFX}.
     * @param id CAN ID of the device, as configured in Phoenix Tuner.
     * @param inverted If the motor is inverted.
     */
    public static SwerveMotor.Ctor talonFX(int id, boolean inverted) {
        return (config, isMoveMotor) -> {
            var deviceLogger = new TalonFXLogger();
            TalonFX talonFX = new TalonFX(id, config.getPhoenixCanBus());
            int PID_SLOT = 0;

            StatusSignal<Double> position = talonFX.getPosition().clone();
            StatusSignal<Double> velocity = talonFX.getVelocity().clone();

            boolean enableFOC = isMoveMotor ? config.getPhoenixMoveFOC() : config.getPhoenixTurnFOC();
            PositionVoltage positionControl = new PositionVoltage(0.0).withSlot(PID_SLOT).withEnableFOC(enableFOC);
            VelocityVoltage velocityControl = new VelocityVoltage(0.0).withSlot(PID_SLOT).withEnableFOC(enableFOC);
            VoltageOut voltageControl = new VoltageOut(0);

            double[] pidGains = isMoveMotor ? config.getMovePID() : config.getTurnPID();
            double[] ffGains = isMoveMotor ? config.getMoveFF() : new double[] { 0.0, 0.0, 0.0 };

            var talonConfig = new TalonFXConfiguration();

            talonConfig.CurrentLimits.StatorCurrentLimit = config.getMoveCurrentLimit();
            talonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            talonConfig.CurrentLimits.SupplyCurrentLimit = config.getMoveCurrentLimit();
            talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

            talonConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            talonConfig.MotorOutput.NeutralMode =
                (isMoveMotor ? config.getMoveBrakeMode() : config.getTurnBrakeMode()) ? NeutralModeValue.Brake : NeutralModeValue.Coast;

            talonConfig.Slot0.kP = pidGains[0];
            talonConfig.Slot0.kI = pidGains[1];
            talonConfig.Slot0.kD = pidGains[2];
            talonConfig.Slot0.kS = ffGains[0];
            talonConfig.Slot0.kV = ffGains[1];
            talonConfig.Slot0.kA = ffGains[2];

            talonFX.getConfigurator().apply(talonConfig);

            BaseStatusSignal.setUpdateFrequencyForAll(1.0 / config.getOdometryPeriod(), position, velocity);
            talonFX.optimizeBusUtilization(1.0 / SwerveBaseHardware.TELEMETRY_CAN_PERIOD, 0.05);

            return new SwerveMotor() {
                @Override
                public double getPosition() {
                    return BaseStatusSignal.getLatencyCompensatedValue(position, velocity);
                }

                @Override
                public void setPosition(double position) {
                    talonFX.setControl(positionControl.withPosition(position));
                }

                @Override
                public double getVelocity() {
                    return velocity.getValue();
                }

                @Override
                public void setVelocity(double velocity) {
                    talonFX.setControl(velocityControl.withVelocity(velocity));
                }

                @Override
                public void setVoltage(double voltage) {
                    talonFX.setControl(voltageControl.withOutput(voltage));
                }

                @Override
                public Object getAPI() {
                    return talonFX;
                }

                @Override
                public void log(DataLogger logger, ErrorHandler errorHandler) {
                    deviceLogger.tryUpdate(logger, talonFX, errorHandler);
                }

                @Override
                public List<BaseStatusSignal> getSignals() {
                    return List.of(position, velocity);
                }

                @Override
                public void close() {
                    talonFX.close();
                }
            };
        };
    }
}
