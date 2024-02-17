package org.team340.lib.swerve.hardware.motors.vendors;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.swerve.util.SwerveConversions;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.PIDConfig;
import org.team340.lib.util.config.rev.RelativeEncoderConfig;
import org.team340.lib.util.config.rev.SparkAbsoluteEncoderConfig;
import org.team340.lib.util.config.rev.SparkFlexConfig;
import org.team340.lib.util.config.rev.SparkFlexConfig.Frame;
import org.team340.lib.util.config.rev.SparkPIDControllerConfig;

/**
 * Wrapper for a REV Spark Flex for swerve.
 */
public class SwerveSparkFlex implements SwerveMotor {

    private static final int PID_SLOT = 0;

    private final CANSparkFlex sparkFlex;
    private final boolean isMoveMotor;
    private final RelativeEncoder relativeEncoder;
    private final SparkPIDController pidController;

    /**
     * Create the Spark Flex wrapper.
     * @param isMoveMotor If the motor is a move motor.
     * @param sparkFlex The Spark Flex to wrap.
     * @param encoder The absolute encoder being used. {@code null} if the motor is a move motor.
     * @param config The general swerve config.
     * @param moduleConfig The motor's module's config.
     */
    public SwerveSparkFlex(
        boolean isMoveMotor,
        CANSparkFlex sparkFlex,
        SwerveEncoder encoder,
        SwerveConfig config,
        SwerveModuleConfig moduleConfig
    ) {
        this.sparkFlex = sparkFlex;
        this.isMoveMotor = isMoveMotor;

        relativeEncoder = sparkFlex.getEncoder();
        pidController = sparkFlex.getPIDController();

        SwerveConversions conversions = new SwerveConversions(config);

        int periodMs = (int) (config.getPeriod() * 1000.0);
        int periodOdometryMs = (int) (config.getOdometryPeriod() * 1000.0);
        boolean usingAttachedEncoder = SwerveEncoder.Type.SPARK_ENCODER.equals(moduleConfig.getEncoderType()) && !isMoveMotor;
        double conversionFactor = 1.0 / (isMoveMotor ? conversions.moveRotationsPerMeter() : conversions.turnRotationsPerRadian());
        PIDConfig pidConfig = isMoveMotor ? config.getMovePID() : config.getTurnPID();

        new SparkFlexConfig()
            .clearFaults()
            .enableVoltageCompensation(config.getOptimalVoltage())
            .setSmartCurrentLimit((int) (isMoveMotor ? config.getMoveCurrentLimit() : config.getTurnCurrentLimit()))
            .setIdleMode(
                (isMoveMotor ? moduleConfig.getMoveMotorBrake() : moduleConfig.getTurnMotorBrake()) ? IdleMode.kBrake : IdleMode.kCoast
            )
            .setInverted(isMoveMotor ? moduleConfig.getMoveMotorInverted() : moduleConfig.getTurnMotorInverted())
            .setOpenLoopRampRate(isMoveMotor ? config.getMoveRampRate() : config.getTurnRampRate())
            .setClosedLoopRampRate(isMoveMotor ? config.getMoveRampRate() : config.getTurnRampRate())
            .setPeriodicFramePeriod(Frame.S0, periodMs)
            .setPeriodicFramePeriod(Frame.S1, periodOdometryMs)
            .setPeriodicFramePeriod(Frame.S2, periodOdometryMs)
            .setPeriodicFramePeriod(Frame.S3, 10000)
            .setPeriodicFramePeriod(Frame.S4, usingAttachedEncoder ? periodOdometryMs : 10000)
            .setPeriodicFramePeriod(Frame.S5, usingAttachedEncoder ? periodOdometryMs : 10000)
            .apply(sparkFlex);

        new SparkPIDControllerConfig()
            .setFeedbackDevice(relativeEncoder)
            .setPID(pidConfig.p(), pidConfig.i(), pidConfig.d(), PID_SLOT)
            .setIZone(pidConfig.iZone(), PID_SLOT)
            .apply(sparkFlex, pidController);

        new RelativeEncoderConfig()
            .setPositionConversionFactor(conversionFactor)
            .setVelocityConversionFactor(conversionFactor / 60.0)
            .apply(sparkFlex, relativeEncoder);

        if (usingAttachedEncoder) {
            new SparkAbsoluteEncoderConfig()
                .setPositionConversionFactor(Math2.TWO_PI)
                .setVelocityConversionFactor(Math2.TWO_PI / 60.0)
                .setInverted(moduleConfig.getEncoderInverted())
                .setZeroOffset(moduleConfig.getEncoderOffset())
                .apply(sparkFlex, sparkFlex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle));
        }

        sparkFlex.set(0.0);
        relativeEncoder.setPosition(0.0);
    }

    @Override
    public double getVelocity() {
        return relativeEncoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return relativeEncoder.getPosition();
    }

    @Override
    public double getDutyCycle() {
        return sparkFlex.getAppliedOutput();
    }

    @Override
    public void setReference(double target, double ff) {
        pidController.setReference(
            target,
            isMoveMotor ? CANSparkFlex.ControlType.kVelocity : CANSparkFlex.ControlType.kPosition,
            PID_SLOT,
            ff,
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setVoltage(double voltage) {
        sparkFlex.setVoltage(voltage);
    }
}
