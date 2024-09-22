package org.team340.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Math utilities.
 * We reinvented math. It's called Math 2.
 */
public final class Math2 {

    private Math2() {
        throw new UnsupportedOperationException("This is a utility class!");
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
    public static final double TWO_THIRDS_PI = 2.0 * Math.PI / 3.0;
    /** {@code 3PI/4} (135deg) */
    public static final double THREE_QUARTERS_PI = 3.0 * Math.PI / 4.0;
    /** {@code 5PI/6} (150deg) */
    public static final double FIVE_SIXTHS_PI = 5.0 * Math.PI / 6.0;
    /** {@code PI*2} (360deg) */
    public static final double TWO_PI = Math.PI * 2.0;
    /** Identity {@link Twist2d} */
    public static final Twist2d TWIST2D_0 = new Twist2d();
    /** Identity {@link ChassisSpeeds} */
    public static final ChassisSpeeds CHASSIS_SPEEDS_0 = new ChassisSpeeds();

    /**
     * Wraps an angle within {@code +-PI} of a reference.
     * @param ref The reference angle in radians.
     * @param angle The angle to wrap in radians.
     * @return The wrapped angle in radians.
     */
    public static double wrapAbout(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) return angle - (Math2.TWO_PI); else if (diff < -Math.PI) return angle + (Math2.TWO_PI); else return angle;
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
     * Definition of a 2D parametric function.
     */
    @FunctionalInterface
    public static interface Parametric {
        public double f(double x, double y);
    }

    /**
     * Finds the root of a 2D parametric function with the false position method (regula falsi).
     * @param func The function to take the root of.
     * @param x0 {@code x} value of the lower bracket.
     * @param y0 {@code y} value of the lower bracket.
     * @param f0 value of {@code func} at {@code x0}, {@code y0}.
     * @param x1 {@code x} value of the upper bracket.
     * @param y1 {@code y} value of the upper bracket.
     * @param f1 value of {@code func} at {@code x1}, {@code y1}.
     * @param iterationsLeft Number of iterations of root finding remaining.
     * @return The parameter value {@code s} that interpolating between {@code 0.0} and {@code 1.0} that corresponds with the approximate root.
     */
    public static double findRoot(Parametric func, double x0, double y0, double f0, double x1, double y1, double f1, int iterationsLeft) {
        if (iterationsLeft < 0 || epsilonEquals(f0, f1)) return 1.0;
        iterationsLeft--;

        double sGuess = Math.max(0.0, Math.min(1.0, -f0 / (f1 - f0)));
        double xGuess = (x1 - x0) * sGuess + x0;
        double yGuess = (y1 - y0) * sGuess + y0;
        double fGuess = func.f(xGuess, yGuess);

        if (Math.signum(f0) == Math.signum(fGuess)) {
            return sGuess + (1.0 - sGuess) * findRoot(func, xGuess, yGuess, fGuess, x1, y1, f1, iterationsLeft);
        } else {
            return sGuess * findRoot(func, x0, y0, f0, xGuess, yGuess, fGuess, iterationsLeft);
        }
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
        return epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon);
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
        return epsilonEquals(a.dx, b.dx, epsilon) && epsilonEquals(a.dy, b.dy, epsilon) && epsilonEquals(a.dtheta, b.dtheta, epsilon);
    }
}
