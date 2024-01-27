package org.team340.lib.swerve.hardware.motors.vendors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.swerve.util.SwerveConversions;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.PIDConfig;

// TODO iZone

/**
 * Wrapper for a Talon FX for swerve.
 */
public class SwerveTalonFX implements SwerveMotor {

    private static final int PID_SLOT = 0;

    private final TalonFX talonFX;
    private final boolean isMoveMotor;
    private final StatusSignal<Double> velocitySignal;
    private final StatusSignal<Double> positionSignal;
    private final StatusSignal<Double> dutyCycleSignal;
    private final double conversionFactor;

    /**
     * Create the Talon FX wrapper.
     * @param isMoveMotor If the motor is a move motor.
     * @param talonFX The Talon FX to wrap.
     * @param config The general swerve config.
     * @param moduleConfig The motor's module's config.
     */
    public SwerveTalonFX(boolean isMoveMotor, TalonFX talonFX, SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.talonFX = talonFX;
        this.isMoveMotor = isMoveMotor;

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

        PIDConfig pidConfig = isMoveMotor ? config.getMovePID() : config.getTurnPID();
        fxConfig.Slot0.kP = pidConfig.p();
        fxConfig.Slot0.kI = pidConfig.i();
        fxConfig.Slot0.kD = pidConfig.d();
        fxConfig.Slot0.kS = 0.0;
        fxConfig.Slot0.kV = 0.0;

        velocitySignal = talonFX.getVelocity();
        positionSignal = talonFX.getPosition();
        dutyCycleSignal = talonFX.getDutyCycle();

        double hz = 1.0 / config.getPeriod();
        velocitySignal.setUpdateFrequency(hz);
        positionSignal.setUpdateFrequency(hz);
        dutyCycleSignal.setUpdateFrequency(hz);

        talonFX.clearStickyFaults();
        talonFX.getConfigurator().apply(fxConfig);

        talonFX.set(0.0);
        talonFX.setPosition(0.0);
    }

    @Override
    public double getVelocity() {
        return velocitySignal.refresh().getValue() / conversionFactor;
    }

    @Override
    public double getPosition() {
        return positionSignal.refresh().getValue() / conversionFactor;
    }

    @Override
    public double getDutyCycle() {
        return dutyCycleSignal.refresh().getValue();
    }

    @Override
    public void setReference(double target, double ff) {
        if (isMoveMotor) {
            VelocityVoltage request = new VelocityVoltage(target * conversionFactor).withSlot(PID_SLOT).withFeedForward(ff);
            talonFX.setControl(request);
        } else {
            PositionVoltage request = new PositionVoltage(target * 1.0 / Math2.TWO_PI).withSlot(PID_SLOT).withFeedForward(ff);
            talonFX.setControl(request);
        }
    }

    @Override
    public void setVoltage(double voltage) {
        talonFX.setVoltage(voltage);
    }
}
