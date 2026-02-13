// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team340.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants.RobotMap;

public class Shootake extends GRRSubsystem {

    private final SparkMax hopper;
    private final SparkMax shooter;

    /** Creates a new Shootake. */
    public Shootake() {
        hopper = new SparkMax(RobotMap.HOPPER_MOTOR, MotorType.kBrushless);
        shooter = new SparkMax(RobotMap.INTAKE_SHOOTER_MOTOR, MotorType.kBrushless);
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
                shooter.set(.75);
            })
            .withTimeout(.5)
            .andThen(
                commandBuilder()
                    .onExecute(() -> {
                        shooter.set(.75);
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
