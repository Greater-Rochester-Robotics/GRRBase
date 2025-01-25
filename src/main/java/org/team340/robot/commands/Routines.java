package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Intake.IntakeSpeed;
import org.team340.robot.subsystems.Wrist;
import org.team340.robot.subsystems.Wrist.WristPosition;

/**
 * The Routines class contains command compositions, such as sequences
 * or parallel command groups, that require multiple subsystems.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class Routines {

    private final Wrist wrist;
    private final Intake intake;

    public Routines(Robot robot) {
        wrist = robot.wrist;
        intake = robot.intake;
    }

    /**
     * Moves the wrist and runs the rollers to intake.
     */
    public Command intake() {
        return parallel(wrist.goTo(WristPosition.kIntake, false), intake.intake()).withName("Routines.intake()");
    }

    private Command shoot(IntakeSpeed shootSpeed, WristPosition wristPosition) {
        return sequence(
            deadline(
                parallel(wrist.goTo(wristPosition).asProxy(), waitSeconds(shootSpeed.spinTime)),
                intake.prepShoot(shootSpeed)
            ),
            intake.shoot(shootSpeed)
        ).withName("Routines.shoot(" + shootSpeed.name() + ", " + wristPosition.name() + ")");
    }

    /**
     * Shoots the configured short distance.
     */
    public Command shootShort() {
        return shoot(IntakeSpeed.kShootShort, WristPosition.kShootShort);
    }

    /**
     * Shoots the configured medium distance.
     */
    public Command shootMedium() {
        return shoot(IntakeSpeed.kShootMedium, WristPosition.kShootMedium);
    }

    /**
     * Shoots the configured far distance.
     */
    public Command shootFar() {
        return shoot(IntakeSpeed.kShootFar, WristPosition.kShootFar);
    }

    /**
     * Should be ran while disabled, and cancelled when enabled.
     * Calls {@code onDisable()} for all subsystems.
     */
    public Command onDisable() {
        return parallel(intake.onDisable(), wrist.onDisable()).withName("Routines.onDisable()");
    }
}
