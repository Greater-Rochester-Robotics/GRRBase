package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.FieldFlip;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.Tunable.TunableDouble;
import org.team340.lib.util.command.AutoChooser;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@SuppressWarnings("unused")
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private static final TunableDouble deceleration = Tunable.value("autos/deceleration", 6.0);
    private static final TunableDouble tolerance = Tunable.value("autos/tolerance", 0.05);

    private final Swerve swerve;
    private final Routines routines;

    private final AutoChooser chooser;

    public Autos(Robot robot) {
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto chooser
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.add("Example", example());
    }

    private Command example() {
        Pose2d start = new Pose2d(1.0, 1.0, Rotation2d.kZero);
        Pose2d firstGoal = new Pose2d(2.0, 5.0, Rotation2d.k180deg);
        Pose2d secondGoal = new Pose2d(6.0, 2.5, Rotation2d.kCW_90deg);

        return sequence(
            swerve.resetPose(FieldFlip.allianceDiagonal(start)),
            swerve.apfDrive(FieldFlip.allianceDiagonal(firstGoal), deceleration::get, tolerance::get),
            swerve.apfDrive(FieldFlip.allianceDiagonal(secondGoal), deceleration::get, tolerance::get),
            swerve.stop(false)
        );
    }
}
