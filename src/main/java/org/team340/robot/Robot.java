package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team340.lib.util.DisableWatchdog;
import org.team340.lib.util.Tunable;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Swerve;

@Logged
public final class Robot extends TimedRobot {

    public final CommandScheduler scheduler = CommandScheduler.getInstance();

    public final Swerve swerve;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;
    private final CommandXboxController coDriver;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        DisableWatchdog.in(scheduler, "m_watchdog");
        DisableWatchdog.in(this, "m_watchdog");

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        Epilogue.getConfig().root = "/Telemetry";

        // Initialize subsystems
        swerve = new Swerve();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize controllers
        driver = new CommandXboxController(Constants.DRIVER);
        coDriver = new CommandXboxController(Constants.CO_DRIVER);

        // Set default commands
        swerve.setDefaultCommand(swerve.drive(this::driverX, this::driverY, this::driverAngular));

        // Driver bindings
        driver.povLeft().onTrue(swerve.tareRotation());

        // Co-driver bindings
        coDriver.a().onTrue(none());

        // Set thread priority
        waitSeconds(10.0)
            .until(DriverStation::isEnabled)
            .andThen(() -> Threads.setCurrentThreadPriority(true, 10))
            .schedule();
    }

    /**
     * Returns the current match time in seconds.
     */
    public double matchTime() {
        return Math.max(0.0, DriverStation.getMatchTime());
    }

    @NotLogged
    public double driverX() {
        return driver.getLeftX();
    }

    @NotLogged
    public double driverY() {
        return driver.getLeftY();
    }

    @NotLogged
    public double driverAngular() {
        return driver.getLeftTriggerAxis() - driver.getRightTriggerAxis();
    }

    @Override
    public void robotPeriodic() {
        scheduler.run();
        Epilogue.update(this);
        Tunable.update();
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}
}
