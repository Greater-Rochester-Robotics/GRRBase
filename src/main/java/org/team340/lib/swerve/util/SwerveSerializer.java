package org.team340.lib.swerve.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.team340.lib.util.Math2;

/**
 * Serializers for swerve helper classes.
 */
public final class SwerveSerializer {

    private SwerveSerializer() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Returns the norm of the provided chassis speeds in meters/second.
     * @param speeds The chassis speeds.
     */
    public static double chassisSpeedsNorm(ChassisSpeeds speeds) {
        return chassisSpeedsNorm(speeds, false);
    }

    /**
     * Returns the norm of the provided chassis speeds in meters/second.
     * @param speeds The chassis speeds.
     * @param toFixed If the returned value should be rounded to a fixed precision.
     */
    public static double chassisSpeedsNorm(ChassisSpeeds speeds, boolean toFixed) {
        return Math2.toFixed(Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
    }

    /**
     * Converts module states into a single array. This is made to be compatible with AdvantageScope.
     * Returns in the following format (heading is in radians):
     * <pre>
     * [
     *   heading0, velocity0,
     *   heading1, velocity1,
     *   ...
     * ]
     * </pre>
     * @param states The states to convert.
     */
    public static double[] moduleStatesDoubleArray(SwerveModuleState[] states) {
        return moduleStatesDoubleArray(states, false);
    }

    /**
     * Converts module states into a single array. This is made to be compatible with AdvantageScope.
     * Returns in the following format (heading is in radians):
     * <pre>
     * [
     *   heading0, velocity0,
     *   heading1, velocity1,
     *   ...
     * ]
     * </pre>
     * @param states The states to convert.
     * @param toFixed If the returned values should be rounded to a fixed precision.
     */
    public static double[] moduleStatesDoubleArray(SwerveModuleState[] states, boolean toFixed) {
        double[] array = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            array[i * 2] = states[i].angle.getRadians();
            array[(i * 2) + 1] = states[i].speedMetersPerSecond;
        }
        return array;
    }
}
