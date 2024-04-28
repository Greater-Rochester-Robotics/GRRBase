package org.team340.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.team340.lib.swerve.SwerveBase;
import org.team340.robot.Constants.SwerveConstants;

/**
 * The swerve subsystem.
 */
public class Swerve extends SwerveBase {

    /**
     * Create the swerve subsystem.
     */
    public Swerve() {
        super("Swerve Drive", SwerveConstants.CONFIG);
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
     * Drives the robot.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param rot The desired rotational speed from {@code -1.0} to {@code 1.0}.
     * @param fieldRelative If the robot should drive field relative.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> rot, boolean fieldRelative) {
        return commandBuilder("swerve.drive()").onExecute(() -> drive(x.get(), y.get(), rot.get(), fieldRelative));
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
