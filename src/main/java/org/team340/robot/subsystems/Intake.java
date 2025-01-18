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
        HOLD_INNER(0.05),
        INTAKE_OUTER(0.4),
        INTAKE_INNER(0.3),
        SHOOT_SHORT(-0.25, 0.6),
        SHOOT_MEDIUM(-0.3, 0.7),
        SHOOT_FAR(-1.0, 2.0),
        SHOOT_INNER(-1.0);

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
        // IntakeConstants.Configs.UPPER_MOTOR.apply(upperMotor);
        // IntakeConstants.Configs.LOWER_MOTOR.apply(lowerMotor);
        // IntakeConstants.Configs.INNER_MOTOR.apply(innerMotor);

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
     * Sets the rollers to hold. Does not end.
     */
    public Command hold() {
        return commandBuilder("intake.hold()")
            .onInitialize(() -> {
                upperMotor.stopMotor();
                innerMotor.set(IntakeSpeed.HOLD_INNER.value);
            })
            .onEnd(() -> {
                innerMotor.stopMotor();
            });
    }

    /**
     * Sets the rollers to intake. Does not end.
     */
    public Command intake() {
        return commandBuilder("intake.intake()")
            .onInitialize(() -> {
                upperMotor.set(IntakeSpeed.INTAKE_OUTER.value);
                innerMotor.set(IntakeSpeed.INTAKE_INNER.value);
            })
            .onEnd(() -> {
                upperMotor.stopMotor();
                lowerMotor.stopMotor();
                innerMotor.stopMotor();
            });
    }

    /**
     * Sets the outer rollers to shoot, while keeping the inner rollers stopped. Does not end.
     * @param shootSpeed The speed to apply to the outer rollers.
     */
    public Command prepShoot(IntakeSpeed shootSpeed) {
        return commandBuilder("intake.shoot(" + shootSpeed + ")")
            .onInitialize(() -> {
                upperMotor.set(shootSpeed.value);
                innerMotor.set(IntakeSpeed.HOLD_INNER.value);
            })
            .onEnd(() -> {
                upperMotor.stopMotor();
                lowerMotor.stopMotor();
            });
    }

    /**
     * Sets the rollers to shoot. Does not end.
     * @param shootSpeed The speed to apply to the outer rollers.
     */
    public Command shoot(IntakeSpeed shootSpeed) {
        return commandBuilder("intake.shoot(" + shootSpeed + ")")
            .onInitialize(() -> {
                upperMotor.set(shootSpeed.value);
                innerMotor.set(IntakeSpeed.SHOOT_INNER.value);
            })
            .onEnd(() -> {
                upperMotor.stopMotor();
                lowerMotor.stopMotor();
                innerMotor.stopMotor();
            });
    }

    /**
     * Should be ran while disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                upperMotor.stopMotor();
                lowerMotor.stopMotor();
                innerMotor.stopMotor();
            })
            .ignoringDisable(true)
            .withName("intake.onDisable()");
    }
}
