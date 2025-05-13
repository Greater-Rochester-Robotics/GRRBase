package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.AutoChooser;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private final Swerve swerve;
    private final Routines routines;

    private final AutoChooser chooser;

    public Autos(Robot robot) {
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto chooser
        chooser = new AutoChooser("Autos");
        chooser.bind(robot.scheduler);

        // Add autonomous modes to the dashboard
        chooser.add("Example", example());
    }

    private Command example() {
        return sequence(
            swerve.resetPose(() -> new Pose2d(1.0, 1.0, Rotation2d.kZero)),
            swerve.apfDrive(() -> new Pose2d(2.0, 5.0, Rotation2d.kPi), () -> 6.0, () -> 0.02),
            swerve.apfDrive(() -> new Pose2d(6.0, 2.5, Rotation2d.kCW_Pi_2), () -> 6.0, () -> 0.02),
            swerve.stop(false)
        );
    }
}
