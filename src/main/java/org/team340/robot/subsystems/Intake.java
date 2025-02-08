package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.lib.util.vendors.RevUtil;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The intake subsystem.
 * Responsible for controlling the intake rollers.
 */
public class Intake extends GRRSubsystem {

    public enum IntakeSpeed {
        kHoldInner(0.05),
        kIntakeOuter(0.4),
        kIntakeInner(0.3),
        kShootShort(-0.25, 0.6),
        kShootMedium(-0.3, 0.7),
        kShootFar(-1.0, 2.0),
        kShootInner(-1.0);

        public final double value;
        public final double spinTime;

        private IntakeSpeed(double value) {
            this(value, 0.0);
        }

        private IntakeSpeed(double value, double spinTime) {
            this.value = value;
            this.spinTime = spinTime;
        }
    }

    private final SparkMax upperMotor;
    private final SparkMax lowerMotor;
    private final SparkMax innerMotor;

    public Intake() {
        upperMotor = new SparkMax(RobotMap.kUpperMotor, MotorType.kBrushless);
        lowerMotor = new SparkMax(RobotMap.kLowerMotor, MotorType.kBrushless);
        innerMotor = new SparkMax(RobotMap.kInnerMotor, MotorType.kBrushless);

        SparkMaxConfig upperConfig = new SparkMaxConfig();

        upperConfig
            .voltageCompensation(Constants.kVoltage)
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.3)
            .openLoopRampRate(0.8);

        upperConfig.signals
            .absoluteEncoderPositionPeriodMs(20)
            .absoluteEncoderVelocityPeriodMs(20)
            .analogPositionPeriodMs(20)
            .analogVelocityPeriodMs(20)
            .analogVoltagePeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .faultsPeriodMs(250)
            .iAccumulationPeriodMs(20)
            .limitsPeriodMs(20)
            .motorTemperaturePeriodMs(1000)
            .outputCurrentPeriodMs(20)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(20)
            .warningsPeriodMs(250);

        upperConfig.closedLoop.pid(1.85, 0.0, 0.3).iZone(0.0).positionWrappingEnabled(false);

        RevUtil.config(upperMotor, upperConfig);

        SparkMaxConfig lowerConfig = new SparkMaxConfig();

        lowerConfig
            .voltageCompensation(Constants.kVoltage)
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .closedLoopRampRate(0.3)
            .openLoopRampRate(0.8)
            .follow(upperMotor, true);

        lowerConfig.signals
            .absoluteEncoderPositionPeriodMs(20)
            .absoluteEncoderVelocityPeriodMs(20)
            .analogPositionPeriodMs(20)
            .analogVelocityPeriodMs(20)
            .analogVoltagePeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .faultsPeriodMs(250)
            .iAccumulationPeriodMs(20)
            .limitsPeriodMs(20)
            .motorTemperaturePeriodMs(1000)
            .outputCurrentPeriodMs(20)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(20)
            .warningsPeriodMs(250);

        lowerConfig.closedLoop.pid(1.85, 0.0, 0.3).iZone(0.0).positionWrappingEnabled(false);

        RevUtil.config(lowerMotor, lowerConfig);

        SparkMaxConfig innerConfig = new SparkMaxConfig();

        innerConfig
            .voltageCompensation(Constants.kVoltage)
            .smartCurrentLimit(30)
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .closedLoopRampRate(0.3)
            .openLoopRampRate(0.8);

        innerConfig.signals
            .absoluteEncoderPositionPeriodMs(20)
            .absoluteEncoderVelocityPeriodMs(20)
            .analogPositionPeriodMs(20)
            .analogVelocityPeriodMs(20)
            .analogVoltagePeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .faultsPeriodMs(250)
            .iAccumulationPeriodMs(20)
            .limitsPeriodMs(20)
            .motorTemperaturePeriodMs(1000)
            .outputCurrentPeriodMs(20)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityPeriodMs(20)
            .warningsPeriodMs(250);

        innerConfig.closedLoop.pid(1.85, 0.0, 0.3).iZone(0.0).positionWrappingEnabled(false);

        RevUtil.config(innerMotor, innerConfig);
    }

    /**
     * Sets the rollers to hold.
     */
    public Command hold() {
        return commandBuilder("intake.hold()")
            .onInitialize(() -> {
                upperMotor.stopMotor();
                innerMotor.set(IntakeSpeed.kHoldInner.value);
            })
            .onEnd(() -> {
                innerMotor.stopMotor();
            });
    }

    /**
     * Sets the rollers to intake.
     */
    public Command intake() {
        return commandBuilder("intake.intake()")
            .onInitialize(() -> {
                upperMotor.set(IntakeSpeed.kIntakeOuter.value);
                innerMotor.set(IntakeSpeed.kIntakeInner.value);
            })
            .onEnd(() -> {
                upperMotor.stopMotor();
                lowerMotor.stopMotor();
                innerMotor.stopMotor();
            });
    }

    // *************** Helper Functions ***************

    /**
     * Stops the intake.
     */
    private void stop() {
        upperMotor.stopMotor();
        lowerMotor.stopMotor();
        innerMotor.stopMotor();
    }

    // *************** Commands ***************

    /**
     * Sets the outer rollers to shoot, while keeping the inner rollers stopped. Does not end.
     * @param shootSpeed The speed to apply to the outer rollers.
     */
    public Command prepShoot(IntakeSpeed shootSpeed) {
        return commandBuilder("intake.shoot(" + shootSpeed + ")")
            .onInitialize(() -> {
                upperMotor.set(shootSpeed.value);
                innerMotor.set(IntakeSpeed.kHoldInner.value);
            })
            .onEnd(this::stop);
    }

    /**
     * Sets the rollers to shoot. Does not end.
     * @param shootSpeed The speed to apply to the outer rollers.
     */
    public Command shoot(IntakeSpeed shootSpeed) {
        return commandBuilder("intake.shoot(" + shootSpeed + ")")
            .onInitialize(() -> {
                upperMotor.set(shootSpeed.value);
                innerMotor.set(IntakeSpeed.kShootInner.value);
            })
            .onEnd(this::stop);
    }

    /**
     * Should be ran while disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder().onInitialize(this::stop).ignoringDisable(true).withName("intake.onDisable()");
    }
}
