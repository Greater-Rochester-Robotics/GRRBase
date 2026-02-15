// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team340.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.tunable.TunableTable;
import org.team340.lib.tunable.Tunables;
import org.team340.lib.tunable.Tunables.TunableDouble;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

public class Shootake extends GRRSubsystem {

    private final SparkMax hopper;
    private final SparkMax shooter;

    private static final TunableTable tunables = Tunables.getNested("shooter");
    private static final TunableDouble shootingSpeed = tunables.value("velocity", 2000.0);
    SparkClosedLoopController shooterController;

    /** Creates a new Shootake. */
    public Shootake() {
        hopper = new SparkMax(RobotMap.HOPPER_MOTOR, MotorType.kBrushless);
        shooter = new SparkMax(RobotMap.INTAKE_SHOOTER_MOTOR, MotorType.kBrushless);
        SparkBaseConfig shooterConfig = new SparkMaxConfig();
        shooterConfig
            .voltageCompensation(12)
            .idleMode(IdleMode.kCoast)
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.01, 0, 0)
            .feedForward.kS(0.0)
            .kV(.13);
        shooterConfig.encoder
            .positionConversionFactor(1.0)
            .velocityConversionFactor(1.0)
            .uvwMeasurementPeriod(16)
            .uvwAverageDepth(2);
        SparkBaseConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.voltageCompensation(12).idleMode(IdleMode.kBrake);
        shooter.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        shooterController = shooter.getClosedLoopController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public Command intake() {
        return commandBuilder()
            .onExecute(() -> {
                shooter.set(.5);
                hopper.set(1);
            })
            .onEnd(() -> {
                shooter.stopMotor();
                hopper.stopMotor();
            });
    }

    public Command barf() {
        return commandBuilder()
            .onExecute(() -> {
                shooter.set(-.5);
                hopper.set(-.7);
            })
            .onEnd(() -> {
                shooter.stopMotor();
                hopper.stopMotor();
            });
    }

    public Command shoot() {
        return commandBuilder()
            .onExecute(() -> {
                shooterController.setSetpoint(shootingSpeed.getAsDouble(), ControlType.kVelocity);
            })
            .withTimeout(.5)
            .andThen(
                commandBuilder()
                    .onExecute(() -> {
                        shooterController.setSetpoint(shootingSpeed.getAsDouble(), ControlType.kVelocity);
                        hopper.set(-1);
                    })
                    .onEnd(() -> {
                        shooter.stopMotor();
                        hopper.stopMotor();
                    })
            );
    }

    public Command stop() {
        return commandBuilder().onExecute(() -> {
            shooter.set(0);
            hopper.set(0);
        });
    }
}
