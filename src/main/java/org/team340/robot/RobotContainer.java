package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.choreo.lib.Choreo;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.GRRDashboard;
import org.team340.lib.controller.Controller2;
import org.team340.lib.controller.JoystickProfiler;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.rev.RevConfigUtils;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.commands.Autos;
import org.team340.robot.commands.SystemsCheck;
import org.team340.robot.subsystems.Swerve;

/**
 * This class is used to declare subsystems, commands, and trigger mappings.
 */
public final class RobotContainer {

    private RobotContainer() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static Controller2 driver;
    private static Controller2 coDriver;

    public static Swerve swerve;

    /**
     * Entry to initializing subsystems and command execution.
     */
    public static void init() {
        // Initialize controllers.
        driver = new Controller2(ControllerConstants.DRIVER);
        coDriver = new Controller2(ControllerConstants.CO_DRIVER);

        // Add controllers to the dashboard.
        driver.addToDashboard();
        coDriver.addToDashboard();

        // Initialize subsystems.
        swerve = new Swerve();

        // Add subsystems to the dashboard.
        swerve.addToDashboard();

        // Set systems check command.
        GRRDashboard.setSystemsCheck(SystemsCheck.command());

        // Print successful REV hardware initialization.
        RevConfigUtils.printSuccess();

        // Configure bindings and autos.
        configBindings();
        configAutos();
    }

    /**
     * This method should be used to declare triggers (created with an
     * arbitrary predicate or from controllers) and their bindings.
     */
    private static void configBindings() {
        // Set default commands.
        swerve.setDefaultCommand(swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true));

        /**
         * Driver bindings.
         */

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zeroIMU(Math2.ROTATION2D_0));

        // Left Bumper => Snap 180
        driver.leftBumper().whileTrue(swerve.driveSnap180(RobotContainer::getDriveX, RobotContainer::getDriveY));

        // Right Bumper => Lock wheels
        driver.rightBumper().whileTrue(swerve.lock());

        /**
         * Co-driver bindings.
         */

        // A => Do nothing
        coDriver.a().onTrue(none());

        /**
         * Joystick profiling.
         */
        driver
            .start()
            .and(driver.leftBumper())
            .and(RobotModeTriggers.disabled())
            .whileTrue(JoystickProfiler.run(driver.getHID(), Axis.kLeftX.value, Axis.kLeftY.value, 100));
        driver
            .start()
            .and(driver.rightBumper())
            .and(RobotModeTriggers.disabled())
            .whileTrue(JoystickProfiler.run(driver.getHID(), Axis.kRightX.value, Axis.kRightY.value, 100));
        coDriver
            .start()
            .and(coDriver.leftBumper())
            .and(RobotModeTriggers.disabled())
            .whileTrue(JoystickProfiler.run(coDriver.getHID(), Axis.kLeftX.value, Axis.kLeftY.value, 100));
        coDriver
            .start()
            .and(coDriver.rightBumper())
            .and(RobotModeTriggers.disabled())
            .whileTrue(JoystickProfiler.run(coDriver.getHID(), Axis.kRightX.value, Axis.kRightY.value, 100));
    }

    /**
     * Autonomous commands should be declared here and
     * added to {@link GRRDashboard}.
     */
    private static void configAutos() {
        GRRDashboard.addAutoCommand("Example", Choreo.getTrajectoryGroup("TestPath"), Autos.example());
        GRRDashboard.addAutoCommand("Example 2", Choreo.getTrajectoryGroup("TestPath2"), Autos.example());
    }

    /**
     * Gets the X axis drive speed from the driver's controller.
     */
    private static double getDriveX() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftY(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the Y axis drive speed from the driver's controller.
     */
    private static double getDriveY() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftX(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the rotational drive speed from the driver's controller.
     */
    private static double getDriveRotate() {
        return driver.getTriggerDifference(ControllerConstants.DRIVE_ROT_MULTIPLIER, ControllerConstants.DRIVE_ROT_EXP);
    }
}
