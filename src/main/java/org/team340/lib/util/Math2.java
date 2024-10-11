package org.team340.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

    /** Shared maximum accuracy floating point. */
    public static final double EPSILON = 1e-8;
    /** {@code PI/6} (30deg) */
    public static final double SIXTH_PI = Math.PI / 6.0;
    /** {@code PI/4} (45deg) */
    public static final double QUARTER_PI = Math.PI / 4.0;
    /** {@code PI/3} (60deg) */
    public static final double THIRD_PI = Math.PI / 3.0;
    /** {@code PI/2} (90deg) */
    public static final double HALF_PI = Math.PI / 2.0;
    /** {@code 2PI/3} (120deg) */
    public static final double TWO_THIRDS_PI = (2.0 * Math.PI) / 3.0;
    /** {@code 3PI/4} (135deg) */
    public static final double THREE_QUARTERS_PI = (3.0 * Math.PI) / 4.0;
    /** {@code 5PI/6} (150deg) */
    public static final double FIVE_SIXTHS_PI = (5.0 * Math.PI) / 6.0;
    /** {@code PI*2} (360deg) */
    public static final double TWO_PI = Math.PI * 2.0;

    public static final Pose2d kZeroPose2d = new Pose2d();
    public static final Rotation2d kZeroRotation2d = new Rotation2d();
    public static final Rotation2d kPiRotation2d = new Rotation2d(Math.PI);

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
     * Check if two values are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    /**
     * Check if two values are equal within the accuracy of a provided epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    /**
     * Checks if two {@link Translation2d}s are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean translationEpsilonEquals(Translation2d a, Translation2d b) {
        return translationEpsilonEquals(a, b, EPSILON);
    }

    /**
     * Checks if two {@link Translation2d}s are equal within the accuracy of a provided epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean translationEpsilonEquals(Translation2d a, Translation2d b, double epsilon) {
        return (epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon));
    }

    /**
     * Checks if two {@link Transform2d}s are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean transformEpsilonEquals(Transform2d a, Transform2d b) {
        return transformEpsilonEquals(a, b, EPSILON);
    }

    /**
     * Checks if two {@link Transform2d}s are equal within the accuracy of a provided epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean transformEpsilonEquals(Transform2d a, Transform2d b, double epsilon) {
        return (
            epsilonEquals(a.getX(), b.getX(), epsilon) &&
            epsilonEquals(a.getY(), b.getY(), epsilon) &&
            epsilonEquals(a.getRotation().getRadians(), b.getRotation().getRadians(), epsilon)
        );
    }

    /**
     * Checks if two {@link Pose2d}s are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean poseEpsilonEquals(Pose2d a, Pose2d b) {
        return poseEpsilonEquals(a, b, EPSILON);
    }

    /**
     * Checks if two {@link Pose2d}s are equal within the accuracy of a provided epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean poseEpsilonEquals(Pose2d a, Pose2d b, double epsilon) {
        return (
            epsilonEquals(a.getX(), b.getX(), epsilon) &&
            epsilonEquals(a.getY(), b.getY(), epsilon) &&
            epsilonEquals(a.getRotation().getRadians(), b.getRotation().getRadians(), epsilon)
        );
    }

    /**
     * Checks if two {@link Twist2d}s are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean twistEpsilonEquals(Twist2d a, Twist2d b) {
        return twistEpsilonEquals(a, b, EPSILON);
    }

    /**
     * Checks if two {@link Twist2d}s are equal within the accuracy of a provided epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean twistEpsilonEquals(Twist2d a, Twist2d b, double epsilon) {
        return (
            epsilonEquals(a.dx, b.dx, epsilon) &&
            epsilonEquals(a.dy, b.dy, epsilon) &&
            epsilonEquals(a.dtheta, b.dtheta, epsilon)
        );
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
