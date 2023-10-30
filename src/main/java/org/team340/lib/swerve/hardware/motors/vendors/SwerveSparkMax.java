package org.team340.lib.swerve.hardware.motors.vendors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import org.team340.lib.swerve.SwerveBase;
import org.team340.lib.swerve.SwerveConversions;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveAbsoluteEncoder;
import org.team340.lib.swerve.hardware.encoders.vendors.SwerveSparkMaxAttachedEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.RevUtil;
import org.team340.lib.util.RevUtil.SparkMax.Frame;

/**
 * Wrapper for a REV Spark Max for swerve.
 */
public class SwerveSparkMax implements SwerveMotor {

    private static final int PID_SLOT = 0;

    /**
     * If the motor is a move motor.
     */
    private final boolean isMoveMotor;
    /**
     * The motor's relative encoder.
     */
    private final RelativeEncoder relativeEncoder;
    /**
     * The integrated PID controller on the Spark Max.
     * {@code null} if a {@link SwerveCANcoder} is in use.
     */
    private final SparkMaxPIDController pidController;

    /**
     * Periodic frames in use by the Spark Max.
     */
    private Frame[] use;

    /**
     * Create the Spark Max wrapper.
     * @param sparkMax The Spark Max to wrap.
     * @param absoluteEncoder The absolute encoder being used. {@code null} if the motor is a move motor.
     * @param config The general swerve config.
     * @param moduleConfig The motor's module's config.
     * @param subsystem The {@link SwerveBase} subsystem in use.
     */
    public SwerveSparkMax(
        CANSparkMax sparkMax,
        SwerveAbsoluteEncoder absoluteEncoder,
        SwerveConfig config,
        SwerveModuleConfig moduleConfig
    ) {
        isMoveMotor = absoluteEncoder == null;
        relativeEncoder = sparkMax.getEncoder();
        pidController = sparkMax.getPIDController();

        SwerveConversions conversions = new SwerveConversions(config);

        RevUtil.SparkMax.restoreFactoryDefaults(sparkMax);
        RevUtil.SparkMax.enableVoltageCompensation(sparkMax, config.getOptimalVoltage());
        RevUtil.SparkMax.setSmartCurrentLimit(sparkMax, (int) (isMoveMotor ? config.getMoveCurrentLimit() : config.getTurnCurrentLimit()));

        RevUtil.SparkMax.setIdleMode(
            sparkMax,
            (isMoveMotor ? moduleConfig.getMoveMotorBrake() : moduleConfig.getTurnMotorBrake()) ? IdleMode.kBrake : IdleMode.kCoast
        );
        RevUtil.SparkMax.setInverted(sparkMax, isMoveMotor ? moduleConfig.getMoveMotorInverted() : moduleConfig.getTurnMotorInverted());

        RevUtil.SparkMax.setOpenLoopRampRate(sparkMax, isMoveMotor ? config.getMoveRampRate() : config.getTurnRampRate());
        RevUtil.SparkMax.setClosedLoopRampRate(sparkMax, isMoveMotor ? config.getMoveRampRate() : config.getTurnRampRate());

        RevUtil.PIDController.setFeedbackDevice(sparkMax, relativeEncoder);

        double[] pid = isMoveMotor ? config.getMovePID() : config.getTurnPID();
        RevUtil.PIDController.setPID(sparkMax, pid[0], pid[1], pid[2], PID_SLOT);

        use = new Frame[] { Frame.S1, Frame.S2 };

        if (isMoveMotor) {
            RevUtil.RelativeEncoder.setPositionConversionFactor(sparkMax, relativeEncoder, 1.0 / conversions.moveRotationsPerMeter());
            RevUtil.RelativeEncoder.setVelocityConversionFactor(
                sparkMax,
                relativeEncoder,
                (1.0 / conversions.moveRotationsPerMeter()) / 60.0
            );
        } else {
            RevUtil.RelativeEncoder.setPositionConversionFactor(sparkMax, relativeEncoder, 1.0 / conversions.turnRotationsPerRadian());
            RevUtil.RelativeEncoder.setVelocityConversionFactor(
                sparkMax,
                relativeEncoder,
                (1.0 / conversions.turnRotationsPerRadian()) / 60.0
            );

            switch (moduleConfig.getAbsoluteEncoderType()) {
                case CANCODER:
                    break;
                case SPARK_MAX_ATTACHED:
                    ((SwerveSparkMaxAttachedEncoder) absoluteEncoder).config(sparkMax);
                    use = new Frame[] { Frame.S1, Frame.S2, Frame.S5, Frame.S6 };
                    break;
            }
        }

        RevUtil.SparkMax.setPeriodicFramePeriods(sparkMax, use);
        RevUtil.SparkMax.clearFaults(sparkMax);
        RevUtil.SparkMax.burnFlash(sparkMax);

        sparkMax.set(0.0);
        relativeEncoder.setPosition(0.0);
    }

    @Override
    public boolean isMoveMotor() {
        return isMoveMotor;
    }

    @Override
    public double getVelocity() {
        return relativeEncoder.getVelocity();
    }

    @Override
    public double getRelativePosition() {
        return relativeEncoder.getPosition();
    }

    @Override
    public void setReference(double target, double ff) {
        pidController.setReference(
            target,
            isMoveMotor ? CANSparkMax.ControlType.kVelocity : CANSparkMax.ControlType.kPosition,
            PID_SLOT,
            isMoveMotor ? ff : 0.0,
            ArbFFUnits.kVoltage
        );
    }
}
