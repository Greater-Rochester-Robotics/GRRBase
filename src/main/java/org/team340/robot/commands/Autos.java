package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    private final AutoFactory factory;
    private final AutoChooser chooser;

    public Autos(Robot robot) {
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto factory
        factory = new AutoFactory(swerve::getPose, swerve::resetPose, swerve::followTrajectory, true, swerve);
        chooser = new AutoChooser();

        // Add autonomous modes to the dashboard
        chooser.addRoutine("Example", this::example);
        SmartDashboard.putData("autos", chooser);
    }

    /**
     * Returns a command that when scheduled will run the currently selected auto.
     */
    public Command runSelectedAuto() {
        return chooser.selectedCommandScheduler();
    }

    private AutoRoutine example() {
        AutoRoutine routine = factory.newRoutine("Example");
        AutoTrajectory exampleTraj = routine.trajectory("ExampleTrajectory");

        routine.active().onTrue(sequence(exampleTraj.resetOdometry(), exampleTraj.spawnCmd()));

        return routine;
    }
}
