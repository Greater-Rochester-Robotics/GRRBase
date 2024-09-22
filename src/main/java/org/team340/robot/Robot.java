package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team340.lib.controller.Controller;
import org.team340.lib.dashboard.GRRDashboard;
import org.team340.lib.rev.RevConfigRegistry;
import org.team340.lib.util.Profiler;
import org.team340.robot.subsystems.Swerve;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public final class Robot extends TimedRobot {

    private final Controller driver;
    private final Controller coDriver;

    private final Swerve swerve;

    private Command autoCommand;

    public Robot() {
        super(Constants.PERIOD);
        DriverStation.silenceJoystickConnectionWarning(true);

        // Start logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        Epilogue.configure(config -> {
            config.root = "Telemetry";
        });

        // Create controllers
        driver = new Controller(Constants.DRIVER);
        coDriver = new Controller(Constants.CO_DRIVER);

        // Initialize Subsystems
        swerve = new Swerve();

        // Finish configuration of REV hardware
        RevConfigRegistry.burnFlashAll();

        // Configure Autos
        GRRDashboard.addAuto("Example", none());

        // A => Do nothing
        driver.a().onTrue(none());

        // A => Do nothing
        coDriver.a().onTrue(none());
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("RobotPeriodic");
        Profiler.run("CommandScheduler", () -> CommandScheduler.getInstance().run());
        Profiler.run("GRRDashboard", GRRDashboard::update);
        Profiler.run("Epilogue", () -> Epilogue.update(this));
        Profiler.end();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autoCommand = GRRDashboard.getSelectedAuto();
        CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autoCommand != null) autoCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
