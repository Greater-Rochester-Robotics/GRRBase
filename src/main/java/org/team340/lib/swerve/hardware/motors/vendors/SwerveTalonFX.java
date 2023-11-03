package org.team340.lib.swerve.hardware.motors.vendors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team340.lib.math.Math2;
import org.team340.lib.swerve.SwerveConversions;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveAbsoluteEncoder;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;

/**
 * Wrapper for a Talon FX for swerve.
 */
public class SwerveTalonFX implements SwerveMotor {

    /**
     * The Talon FX.
     */
    private final TalonFX talonFX;
    /**
     * If the motor is a move motor.
     */
    private final boolean isMoveMotor;
    /**
     * A conversion factor from rotations to meters or radians, depending on the motor type.
     */
    private final double conversionFactor;

    /**
     * Create the Talon FX wrapper.
     * @param talonFX The Talon FX to wrap.
     * @param absoluteEncoder The absolute encoder being used. {@code null} if the motor is a move motor.
     * @param config The general swerve config.
     * @param moduleConfig The motor's module's config.
     */
    public SwerveTalonFX(TalonFX talonFX, SwerveAbsoluteEncoder absoluteEncoder, SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.talonFX = talonFX;
        isMoveMotor = absoluteEncoder == null;

        SwerveConversions conversions = new SwerveConversions(config);
        conversionFactor = isMoveMotor ? conversions.moveRotationsPerMeter() : conversions.turnRotationsPerRadian();

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();

        fxConfig.CurrentLimits.SupplyCurrentLimit = isMoveMotor ? config.getMoveCurrentLimit() : config.getTurnCurrentLimit();
        fxConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        fxConfig.CurrentLimits.StatorCurrentLimit = isMoveMotor ? config.getMoveCurrentLimit() : config.getTurnCurrentLimit();
        fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        fxConfig.MotorOutput.NeutralMode =
            (isMoveMotor ? moduleConfig.getMoveMotorBrake() : moduleConfig.getTurnMotorBrake())
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        fxConfig.MotorOutput.Inverted =
            (isMoveMotor ? moduleConfig.getMoveMotorInverted() : moduleConfig.getTurnMotorInverted())
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        fxConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = isMoveMotor ? config.getMoveRampRate() : config.getTurnRampRate();
        fxConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = isMoveMotor ? config.getMoveRampRate() : config.getTurnRampRate();

        double[] pid = isMoveMotor ? config.getMovePID() : config.getTurnPID();
        fxConfig.Slot0.kP = pid[0];
        fxConfig.Slot0.kI = pid[1];
        fxConfig.Slot0.kD = pid[2];
        fxConfig.Slot0.kS = 0.0;
        fxConfig.Slot0.kV = 0.0;

        double hz = 1.0 / config.getPeriod();
        talonFX.getVelocity().setUpdateFrequency(hz);
        talonFX.getPosition().setUpdateFrequency(hz);

        talonFX.clearStickyFaults();
        talonFX.getConfigurator().apply(fxConfig);

        talonFX.set(0.0);
        talonFX.setPosition(0.0);
    }

    @Override
    public boolean isMoveMotor() {
        return isMoveMotor;
    }

    @Override
    public double getVelocity() {
        return talonFX.getVelocity().getValue() / conversionFactor;
    }

    @Override
    public double getRelativePosition() {
        return talonFX.getPosition().getValue() / conversionFactor;
    }

    @Override
    public void setReference(double target, double ff) {
        if (isMoveMotor) {
            VelocityVoltage request = new VelocityVoltage(target * conversionFactor).withSlot(0).withFeedForward(ff);

            talonFX.setControl(request);
        } else {
            PositionVoltage request = new PositionVoltage(target * 1.0 / Math2.TWO_PI).withSlot(0);
            talonFX.setControl(request);
        }
    }
}
