package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.dashboard.GRRDashboard;
import org.team340.robot.Robot;
import org.team340.robot.subsystems.Foo;
import org.team340.robot.subsystems.Swerve;

/**
 * The Autos class declares autonomous modes, and adds them
 * to the dashboard to be selected by the drive team.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class Autos {

    private final Foo foo;
    private final Swerve swerve;
    private final Routines routines;

    private final AutoFactory factory;

    public Autos(Robot robot) {
        foo = robot.foo;
        swerve = robot.swerve;
        routines = robot.routines;

        // Create the auto factory
        factory = Choreo.createAutoFactory(
            swerve::getPose,
            swerve::choreoController,
            () -> true,
            swerve,
            new AutoBindings()
        );

        // Add autonomous modes to the dashboard
        GRRDashboard.setTrajectoryCache(factory.cache());
        GRRDashboard.addAuto("Example", "ExampleAuto", example());
    }

    /**
     * Resets the pose of the robot to the start of the provided trajectory.
     * @param trajectoryName The name of the trajectory to reset the robot's pose to.
     */
    private Command resetPose(String trajectoryName) {
        var trajectory = factory.newRoutine("void").trajectory(trajectoryName);
        return either(
            swerve.resetPose(() -> trajectory.getInitialPose().get()),
            print("Unable to retrieve initial pose from trajectory \"" + trajectoryName + "\"").andThen(idle()),
            () -> trajectory.getInitialPose().isPresent()
        ).withName("Autos.resetPose(\"" + trajectoryName + "\")");
    }

    private Command example() {
        return sequence(
            resetPose("ExampleAuto"),
            parallel(factory.trajectoryCommand("ExampleAuto"), foo.bar()),
            routines.example(),
            swerve.finishAuto()
        ).withName("Autos.example()");
    }
}
