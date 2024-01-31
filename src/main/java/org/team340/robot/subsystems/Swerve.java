package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.team340.lib.swerve.SwerveBase;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.SwerveConstants;

/**
 * The swerve subsystem.
 */
public class Swerve extends SwerveBase {

    private final ProfiledPIDController rotController = new ProfiledPIDController(
        SwerveConstants.ROT_PID.p(),
        SwerveConstants.ROT_PID.i(),
        SwerveConstants.ROT_PID.d(),
        SwerveConstants.ROT_CONSTRAINTS
    );

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
        rotController.setIZone(SwerveConstants.ROT_PID.iZone());
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        updateOdometry();
    }

    /**
     * Zeroes the IMU to a specified yaw.
     */
    public Command zeroIMU(Rotation2d yaw) {
        return runOnce(() -> imu.setZero(yaw)).withName("swerve.zero(" + yaw.toString() + ")");
    }

    /**
     * Drives the robot as a percent of its max velocity (inputs are from {@code -1.0} to {@code 1.0}).
     * @param x X speed.
     * @param y Y speed.
     * @param rot Rotational speed.
     * @param fieldRelative If the robot should drive field relative.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, boolean fieldRelative) {
        return commandBuilder("swerve.drive()").onExecute(() -> drive(x.get(), y.get(), rot.get(), fieldRelative));
    }

    /**
     * Drives the robot using percents of its calculated max velocity while locked at a field relative angle.
     * @param x X speed.
     * @param y Y speed.
     * @param angle The desired field relative angle to point at in radians.
     */
    public Command driveAngle(Supplier<Double> x, Supplier<Double> y, double angle) {
        return commandBuilder("swerve.driveAngle(" + Math2.toFixed(angle) + ")")
            .onInitialize(() -> rotController.reset(getPosition().getRotation().getRadians(), getVelocity(true).omegaRadiansPerSecond))
            .onExecute(() -> driveAngle(x.get(), y.get(), angle, rotController));
    }

    /**
     * Drives the robot while snapping to facing up or down the field, whichever is closer.
     * Ends when the robot is at the determined angle.
     * @param x X speed.
     * @param y Y speed.
     */
    public Command driveSnap180(Supplier<Double> x, Supplier<Double> y) {
        return either(
            driveAngle(x, y, 0.0),
            driveAngle(x, y, Math.PI),
            () -> Math.abs(MathUtil.angleModulus(imu.getYaw().getRadians())) < Math2.HALF_PI
        )
            .withName("swerve.driveSnap180()");
    }

    /**
     * Drives the modules into an X formation to prevent the robot from moving.
     */
    public Command lock() {
        return commandBuilder("swerve.lock()").onExecute(this::lockWheels);
    }

    /**
     * Runs a SysId quasistatic test.
     * @param direction The direction to run the test in.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Runs a SysId dynamic test.
     * @param direction The direction to run the test in.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
