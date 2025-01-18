package org.team340.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.util.GRRDashboard;
import org.team340.lib.util.Profiler;
import org.team340.lib.util.Tunable;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.subsystems.Wrist;
import org.team340.robot.subsystems.Wrist.WristPosition;

@Logged
public final class Robot extends TimedRobot {

    public final Swerve swerve;
    public final Wrist wrist;
    public final Intake intake;

    public final Routines routines;
    public final Autos autos;

    private final CommandXboxController driver;

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure logging
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        SignalLogger.enableAutoLogging(false);
        Epilogue.getConfig().root = "/Telemetry";

        // Initialize subsystems
        swerve = new Swerve();
        wrist = new Wrist();
        intake = new Intake();

        // Initialize compositions
        routines = new Routines(this);
        autos = new Autos(this);

        // Initialize controller
        driver = new CommandXboxController(Constants.kDriver);

        // Set default commands
        intake.setDefaultCommand(intake.hold());
        swerve.setDefaultCommand(
            swerve.drive(
                driver::getLeftX,
                driver::getLeftY,
                () -> driver.getLeftTriggerAxis() - driver.getRightTriggerAxis()
            )
        );

        // Create triggers
        RobotModeTriggers.autonomous().whileTrue(GRRDashboard.runSelectedAuto());

        // Driver bindings

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.tareRotation());

        // A => Intake (Hold)
        driver.a().whileTrue(routines.intake()).onFalse(wrist.goTo(WristPosition.kSafe));

        // B => Shoot short (Hold)
        driver.b().whileTrue(routines.shootShort()).onFalse(wrist.goTo(WristPosition.kSafe));

        // X => Shoot medium (Hold)
        driver.x().whileTrue(routines.shootMedium()).onFalse(wrist.goTo(WristPosition.kSafe));

        // Y => Shoot far (Hold)
        driver.y().whileTrue(routines.shootFar()).onFalse(wrist.goTo(WristPosition.kSafe));
    }

    @Override
    public void robotPeriodic() {
        Profiler.start("RobotPeriodic");
        Profiler.run("CommandScheduler", () -> CommandScheduler.getInstance().run());
        Profiler.run("Epilogue", () -> Epilogue.update(this));
        Profiler.run("GRRDashboard", GRRDashboard::update);
        Profiler.run("Tunables", Tunable::update);
        Profiler.end();
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
