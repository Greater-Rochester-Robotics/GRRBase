package org.team340.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team340.lib.GRRDashboard;
import org.team340.lib.util.config.rev.RevConfigRegistry;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {

    public Robot() {
        super(Constants.PERIOD);
    }

    @Override
    public void robotInit() {
        LiveWindow.setEnabled(false);
        enableLiveWindowInTest(false);
        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        DataLogManager.logNetworkTables(true);
        GRRDashboard.initAsync(this, Constants.TELEMETRY_PERIOD, Constants.POWER_USAGE_PERIOD);
        RevConfigRegistry.init(this);
        RobotContainer.init();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        GRRDashboard.getAutoCommand().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
