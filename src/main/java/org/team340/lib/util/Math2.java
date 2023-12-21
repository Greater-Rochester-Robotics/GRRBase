package org.team340.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Math utilities.
 * We reinvented math. It's called Math 2.
 */
public final class Math2 {

    private Math2() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Shared maximum accuracy floating point.
     */
    public static final double DEFAULT_EPSILON = 1e-9;
    /**
     * {@code PI * 2}
     */
    public static final double TWO_PI = Math.PI * 2.0;
    /**
     * {@code PI / 2}
     */
    public static final double HALF_PI = Math.PI / 2.0;
    /**
     * {@code PI / 4}
     */
    public static final double QUARTER_PI = Math.PI / 4.0;

    /**
     * Identity {@link Translation2d}.
     */
    public static final Translation2d TRANSLATION2D_0 = new Translation2d();
    /**
     * Identity {@link Translation3d}.
     */
    public static final Translation3d TRANSLATION3D_0 = new Translation3d();
    /**
     * Identity {@link Rotation2d}.
     */
    public static final Rotation2d ROTATION2D_0 = new Rotation2d();
    /**
     * A {@link Rotation2d} with a value of {@code PI}.
     */
    public static final Rotation2d ROTATION2D_PI = new Rotation2d(Math.PI);
    /**
     * Identity {@link Rotation3d}.
     */
    public static final Rotation3d ROTATION3D_0 = new Rotation3d();
    /**
     * Identity {@link Transform2d}.
     */
    public static final Transform2d TRANSFORM2D_0 = new Transform2d();
    /**
     * Identity {@link Transform3d}.
     */
    public static final Transform3d TRANSFORM3D_0 = new Transform3d();
    /**
     * Identity {@link Twist2d}.
     */
    public static final Twist2d TWIST2D_0 = new Twist2d();
    /**
     * Identity {@link Twist3d}.
     */
    public static final Twist3d TWIST3D_0 = new Twist3d();
    /**
     * Identity {@link Pose2d}.
     */
    public static final Pose2d POSE2D_0 = new Pose2d();
    /**
     * Identity {@link Pose3d}.
     */
    public static final Pose3d POSE3D_0 = new Pose3d();
    /**
     * Identity {@link ChassisSpeeds}.
     */
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
     * Check if two values are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, DEFAULT_EPSILON);
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
     * Checks if two {@link Twist2d}s are equal within the accuracy of the default epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @return {@code true} if the values are equal.
     */
    public static boolean twist2dEpsilonEquals(Twist2d a, Twist2d b) {
        return twist2dEpsilonEquals(a, b, DEFAULT_EPSILON);
    }

    /**
     * Checks if two {@link Twist2d}s are equal within the accuracy of a provided epsilon.
     * @param a The first value to compare.
     * @param b The second value to compare.
     * @param epsilon Epsilon value to compare with.
     * @return {@code true} if the values are equal.
     */
    public static boolean twist2dEpsilonEquals(Twist2d a, Twist2d b, double epsilon) {
        return epsilonEquals(a.dx, b.dx, epsilon) && epsilonEquals(a.dy, b.dy, epsilon) && epsilonEquals(a.dtheta, b.dtheta, epsilon);
    }

    /**
     * Rounds a value to a fixed point of 3 decimal places.
     * @param value The value to round.
     * @return The rounded value.
     */
    public static double toFixed(double value) {
        return toFixed(value, 1e-3);
    }

    /**
     * Rounds a value to a fixed point.
     * @param value The value to round.
     * @param precision The fixed point precision to round to as a decimal. For example, {@code 1e-3} rounds to 3 decimal places. This must be a power of {@code 10}.
     * @return The rounded value.
     */
    public static double toFixed(double value, double precision) {
        return Math.round(value / precision) * precision;
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
}
