package org.team340.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.GRRDashboard;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private final Swerve swerve;

    private final AutoFactory factory;

    public Autos(Robot robot) {
        swerve = robot.swerve;

        // Create the auto factory
        factory = new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);

        // Add autonomous modes to the dashboard
        GRRDashboard.setTrajectoryCache(factory.cache());
        GRRDashboard.addAuto("Example", "example", example());
    }

    private Command example() {
        AutoRoutine routine = factory.newRoutine("Example");

        return routine.cmd();
    }
}
