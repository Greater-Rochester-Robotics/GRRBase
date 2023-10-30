package org.team340.lib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Factory for generating chassis speeds.
 */
public class SwerveSpeedsFactory {

    private final SwerveBase swerve;

    /**
     * Create the chassis speeds factory.
     * @param swerve The swerve subsystem.
     */
    public SwerveSpeedsFactory(SwerveBase swerve) {
        this.swerve = swerve;
    }

    /**
     * Drives the robot using percents of its calculated max velocity.
     * Calculated speeds are converted to discrete speeds (from continuous) then are converted to module states using the target controller.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     * @param rot The desired rotational speed from {@code -1.0} to {@code 1.0}.
     * @param fieldRelative If the robot should drive field relative.
     */
    public ChassisSpeeds percent(double x, double y, double rot, boolean fieldRelative) {
        return velocity(x * swerve.config.getMaxV(), y * swerve.config.getMaxV(), rot * swerve.config.getMaxRv(), fieldRelative);
    }

    /**
     * Drives the robot using velocity.
     * Calculated speeds are converted to discrete speeds (from continuous) then are converted to module states using the target controller.
     * @param xV The desired {@code x} velocity in meters/second.
     * @param yV The desired {@code y} velocity in meters/second.
     * @param rotV The desired rotational velocity in radians/second.
     * @param fieldRelative If the robot should drive field relative.
     */
    public ChassisSpeeds velocity(double xV, double yV, double rotV, boolean fieldRelative) {
        return fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xV, yV, rotV, swerve.getYaw()) : new ChassisSpeeds(xV, yV, rotV);
    }

    /**
     * Drives the robot using percents of its calculated max velocity while locked pointing at a position on the field.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     * @param point The desired field relative position to point at (axis values in meters).
     * @param controller A profiled PID controller to use for translating to and maintaining the angle to the desired point.
     */
    public ChassisSpeeds aroundPoint(double x, double y, Translation2d point, ProfiledPIDController controller) {
        return aroundPointVelocity(x * swerve.config.getMaxV(), y * swerve.config.getMaxV(), point, controller);
    }

    /**
     * Drives the robot using velocity while locked pointing at a position on the field.
     * @param xV The desired {@code x} velocity in meters/second.
     * @param yV The desired {@code y} velocity in meters/second.
     * @param point The desired field relative position to point at (axis values in meters).
     * @param controller A profiled PID controller to use for translating to and maintaining the angle to the desired point.
     */
    public ChassisSpeeds aroundPointVelocity(double xV, double yV, Translation2d point, ProfiledPIDController controller) {
        Translation2d robotPoint = swerve.getPosition().getTranslation();
        double angle = MathUtil.angleModulus(point.minus(robotPoint).getAngle().getRadians());
        return angleVelocity(xV, yV, angle, controller);
    }

    /**
     * Drives the robot using percents of its calculated max velocity while locked at a field relative angle.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     * @param angle The desired field relative angle to point at in radians.
     * @param controller A profiled PID controller to use for translating to and maintaining the angle.
     */
    public ChassisSpeeds angle(double x, double y, double angle, ProfiledPIDController controller) {
        return angleVelocity(x * swerve.config.getMaxV(), y * swerve.config.getMaxV(), angle, controller);
    }

    /**
     * Drives the robot using velocity while locked at a field relative angle.
     * @param xV The desired {@code x} velocity in meters/second.
     * @param yV The desired {@code y} velocity in meters/second.
     * @param angle The desired field relative angle to point at in radians.
     * @param controller A profiled PID controller to use for translating to and maintaining the angle.
     */
    public ChassisSpeeds angleVelocity(double xV, double yV, double angle, ProfiledPIDController controller) {
        Rotation2d yaw = swerve.getYaw();
        return ChassisSpeeds.fromFieldRelativeSpeeds(xV, yV, controller.calculate(MathUtil.angleModulus(yaw.getRadians()), angle), yaw);
    }

    /**
     * Drives the robot to a field relative pose.
     * @param pose The pose to drive to.
     * @param xController A PID controller to use for translating to and maintaining pose's {@code x} position.
     * @param yController The PID controller to use for translating to and maintaining pose's {@code y} position.
     * @param rotController The profiled PID controller to use for translating to and maintaining the pose's angle.
     */
    public ChassisSpeeds driveToPose(
        Pose2d pose,
        PIDController xController,
        PIDController yController,
        ProfiledPIDController rotController
    ) {
        Rotation2d yaw = swerve.getYaw();
        Pose2d position = swerve.getPosition();
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(position.getX(), pose.getX()),
            yController.calculate(position.getY(), pose.getY()),
            rotController.calculate(MathUtil.angleModulus(yaw.getRadians()), pose.getRotation().getRadians()),
            yaw
        );
    }
}
