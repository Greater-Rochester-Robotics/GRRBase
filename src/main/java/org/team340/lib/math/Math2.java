package org.team340.lib.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Math utilities.
 * We reinvented math. It's called Math 2.
 */
public final class Math2 {

    private Math2() {
        throw new AssertionError("This is a utility class!");
    }

    public static final double PI2 = 6.283185307179586;

    /**
     * Discretizes a continuous-time chassis speed in place.
     * @param speeds The continuous speeds.
     * @param dtSeconds The duration of the timestep the speeds should be applied for.
     * @return The provided speeds object.
     * @see {@link ChassisSpeeds#discretize(ChassisSpeeds, double)}
     */
    public static ChassisSpeeds discretize(ChassisSpeeds speeds, double dtSeconds) {
        double dtheta = speeds.omegaRadiansPerSecond * dtSeconds;

        double sin = -dtheta / 2.0;
        double cos = Math.abs(Math.cos(dtheta) - 1.0) < 1e-6
            ? 1.0 - ((1.0 / 12.0) * dtheta * dtheta)
            : (sin * Math.sin(dtheta)) / (Math.cos(dtheta) - 1.0);

        double dt = dtSeconds;
        double dx = speeds.vxMetersPerSecond * dt;
        double dy = speeds.vyMetersPerSecond * dt;

        speeds.vxMetersPerSecond = ((dx * cos) - (dy * sin)) / dt;
        speeds.vyMetersPerSecond = ((dx * sin) + (dy * cos)) / dt;

        return speeds;
    }

    /**
     * Checks if the distance between two {@link Translation2d}s is within a specified tolerance.
     * @param expected The expected {@link Translation2d}.
     * @param actual The actual {@link Translation2d}.
     * @param tolerance The allowed distance between the actual and the expected translations, in meters.
     */
    public static boolean isNear(Translation2d expected, Translation2d actual, double tolerance) {
        double dx = expected.getX() - actual.getX();
        double dy = expected.getY() - actual.getY();
        return dx * dx + dy * dy < tolerance * tolerance;
    }

    /**
     * Checks if the angle between two {@link Rotation2d}s is within a specified tolerance.
     * @param expected The expected {@link Rotation2d}.
     * @param actual The actual {@link Rotation2d}.
     * @param tolerance The allowed difference between the actual and the expected rotations, in radians.
     */
    public static boolean isNear(Rotation2d expected, Rotation2d actual, double tolerance) {
        double dot = expected.getCos() * actual.getCos() + expected.getSin() * actual.getSin();
        return Math.acos(MathUtil.clamp(dot, -1.0, 1.0)) < tolerance;
    }

    /**
     * Returns a random double from {@code 0.0} to {@code max}.
     * @param max The maximum value to return.
     */
    public static double random(double max) {
        return Math.random() * max;
    }

    /**
     * Returns a random double from {@code min} to {@code max}.
     * @param min The minimum value to return.
     * @param max The maximum value to return.
     */
    public static double random(double min, double max) {
        return (Math.random() * (max - min)) + min;
    }

    /**
     * Copies values from a source {@link Twist2d} object to another.
     * @param source The twist to copy from.
     * @param output The twist to copy into.
     * @return The output twist.
     */
    public static Twist2d copyInto(Twist2d source, Twist2d output) {
        output.dx = source.dx;
        output.dy = source.dy;
        output.dtheta = source.dtheta;
        return output;
    }

    /**
     * Copies values from a source {@link ChassisSpeeds} object to another.
     * @param source The speeds to copy from.
     * @param output The speeds to copy into.
     * @return The output speeds.
     */
    public static ChassisSpeeds copyInto(ChassisSpeeds source, ChassisSpeeds output) {
        output.vxMetersPerSecond = source.vxMetersPerSecond;
        output.vyMetersPerSecond = source.vyMetersPerSecond;
        output.omegaRadiansPerSecond = source.omegaRadiansPerSecond;
        return output;
    }

    /**
     * Copies values from a source {@link SwerveModulePosition} object to another.
     * @param source The swerve module position to copy from.
     * @param output The swerve module position to copy into.
     * @return The output position.
     */
    public static SwerveModulePosition copyInto(SwerveModulePosition source, SwerveModulePosition output) {
        output.distanceMeters = source.distanceMeters;
        output.angle = source.angle;
        return output;
    }

    /**
     * Copies values from a source {@link SwerveModuleState} object to another.
     * @param source The swerve module state to copy from.
     * @param output The swerve module state to copy into.
     * @return The output state.
     */
    public static SwerveModuleState copyInto(SwerveModuleState source, SwerveModuleState output) {
        output.speedMetersPerSecond = source.speedMetersPerSecond;
        output.angle = source.angle;
        return output;
    }
}
