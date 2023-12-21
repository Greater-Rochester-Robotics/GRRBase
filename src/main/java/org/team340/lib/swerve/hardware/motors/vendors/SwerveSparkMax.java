package org.team340.lib.swerve.hardware.motors.vendors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import org.team340.lib.swerve.SwerveBase.SwerveAbsoluteEncoderType;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.swerve.util.SwerveConversions;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.PIDConfig;
import org.team340.lib.util.config.rev.AbsoluteEncoderConfig;
import org.team340.lib.util.config.rev.RelativeEncoderConfig;
import org.team340.lib.util.config.rev.SparkMaxConfig;
import org.team340.lib.util.config.rev.SparkMaxConfig.Frame;
import org.team340.lib.util.config.rev.SparkMaxPIDControllerConfig;

/**
 * Wrapper for a REV Spark Max for swerve.
 */
public class SwerveSparkMax extends SwerveMotor {

    private static final int PID_SLOT = 0;

    private final RelativeEncoder relativeEncoder;
    private final SparkMaxPIDController pidController;

    /**
     * Create the Spark Max wrapper.
     * @param isMoveMotor If the motor is a move motor.
     * @param sparkMax The Spark Max to wrap.
     * @param encoder The absolute encoder being used. {@code null} if the motor is a move motor.
     * @param config The general swerve config.
     * @param moduleConfig The motor's module's config.
     */
    public SwerveSparkMax(
        boolean isMoveMotor,
        CANSparkMax sparkMax,
        SwerveEncoder encoder,
        SwerveConfig config,
        SwerveModuleConfig moduleConfig
    ) {
        super(isMoveMotor);
        relativeEncoder = sparkMax.getEncoder();
        pidController = sparkMax.getPIDController();

        SwerveConversions conversions = new SwerveConversions(config);

        int periodMs = (int) (config.getPeriod() * 1000.0);
        boolean usingAttachedEncoder = SwerveAbsoluteEncoderType.SPARK_MAX_ENCODER.equals(moduleConfig.getAbsoluteEncoderType());
        double conversionFactor = 1.0 / (isMoveMotor() ? conversions.moveRotationsPerMeter() : conversions.turnRotationsPerRadian());
        PIDConfig pidConfig = isMoveMotor() ? config.getMovePID() : config.getTurnPID();

        new SparkMaxConfig()
            .clearFaults()
            .restoreFactoryDefaults()
            .enableVoltageCompensation(config.getOptimalVoltage())
            .setSmartCurrentLimit((int) (isMoveMotor() ? config.getMoveCurrentLimit() : config.getTurnCurrentLimit()))
            .setIdleMode(
                (isMoveMotor() ? moduleConfig.getMoveMotorBrake() : moduleConfig.getTurnMotorBrake()) ? IdleMode.kBrake : IdleMode.kCoast
            )
            .setInverted(isMoveMotor() ? moduleConfig.getMoveMotorInverted() : moduleConfig.getTurnMotorInverted())
            .setOpenLoopRampRate(isMoveMotor() ? config.getMoveRampRate() : config.getTurnRampRate())
            .setClosedLoopRampRate(isMoveMotor() ? config.getMoveRampRate() : config.getTurnRampRate())
            .setPeriodicFramePeriod(Frame.S0, periodMs)
            .setPeriodicFramePeriod(Frame.S1, periodMs)
            .setPeriodicFramePeriod(Frame.S2, periodMs)
            .setPeriodicFramePeriod(Frame.S3, 10000)
            .setPeriodicFramePeriod(Frame.S4, usingAttachedEncoder ? periodMs : 10000)
            .setPeriodicFramePeriod(Frame.S5, usingAttachedEncoder ? periodMs : 10000)
            .apply(sparkMax);

        new SparkMaxPIDControllerConfig()
            .setFeedbackDevice(relativeEncoder)
            .setPID(pidConfig.p(), pidConfig.i(), pidConfig.d(), PID_SLOT)
            .setIZone(pidConfig.iZone(), PID_SLOT)
            .apply(sparkMax, pidController);

        new RelativeEncoderConfig()
            .setPositionConversionFactor(conversionFactor)
            .setVelocityConversionFactor(conversionFactor / 60.0)
            .apply(sparkMax, relativeEncoder);

        if (usingAttachedEncoder) {
            new AbsoluteEncoderConfig()
                .setPositionConversionFactor(Math2.TWO_PI)
                .setVelocityConversionFactor(Math2.TWO_PI / 60.0)
                .setInverted(moduleConfig.getAbsoluteEncoderInverted())
                .setZeroOffset(moduleConfig.getAbsoluteEncoderOffset())
                .apply(sparkMax, sparkMax.getAbsoluteEncoder(Type.kDutyCycle));
        }

        sparkMax.set(0.0);
        relativeEncoder.setPosition(0.0);
    }

    @Override
    protected double getRealVelocity() {
        return relativeEncoder.getVelocity();
    }

    @Override
    protected double getRealPosition() {
        return relativeEncoder.getPosition();
    }

    @Override
    protected void setRealReference(double target, double ff) {
        pidController.setReference(
            target,
            isMoveMotor() ? CANSparkMax.ControlType.kVelocity : CANSparkMax.ControlType.kPosition,
            PID_SLOT,
            ff,
            ArbFFUnits.kVoltage
        );
    }
}
