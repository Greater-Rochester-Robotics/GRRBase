package org.team340.lib.swerve.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * Helpers for sending swerve visualizations to NT.
 */
public class SwerveVisualizer implements Sendable {

    private static final double[] EMPTY_DOUBLE = new double[0];

    private final Supplier<Pose2d> robot;
    private final Supplier<SwerveModuleState[]> moduleStates;
    private final Supplier<SwerveModuleState[]> desiredModuleStates;

    private Supplier<double[]> target = () -> EMPTY_DOUBLE;
    private double[] trajectory = EMPTY_DOUBLE;
    private double[] trajectoryTarget = EMPTY_DOUBLE;
    private double[] visionMeasurements = EMPTY_DOUBLE;
    private double[] visionTargets = EMPTY_DOUBLE;

    /**
     * Create the visualizer.
     * @param robot A supplier that returns the robot's position.
     * @param moduleStates A supplier that returns module states.
     * @param desiredModuleStates A supplier that returns desired module states.
     */
    public SwerveVisualizer(
        Supplier<Pose2d> robot,
        Supplier<SwerveModuleState[]> moduleStates,
        Supplier<SwerveModuleState[]> desiredModuleStates
    ) {
        this.robot = robot;
        this.moduleStates = moduleStates;
        this.desiredModuleStates = desiredModuleStates;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("robot", () -> pose2d(robot.get()), null);
        builder.addDoubleArrayProperty("moduleStates", () -> moduleStates(moduleStates.get()), null);
        builder.addDoubleArrayProperty("desiredModuleStates", () -> moduleStates(desiredModuleStates.get()), null);
        builder.addDoubleArrayProperty("target", () -> target.get(), null);
        builder.addDoubleArrayProperty("trajectory", () -> trajectory, null);
        builder.addDoubleArrayProperty("trajectoryTarget", () -> trajectoryTarget, null);
        builder.addDoubleArrayProperty("visionMeasurements", () -> visionMeasurements, null);
        builder.addDoubleArrayProperty("visionTargets", () -> visionTargets, null);
    }

    /**
     * Adds a trajectory.
     * @param trajectory The trajectory to be added.
     * @return A consumer that accepts the current state of the trajectory's path following.
     * The first argument is a boolean that should be {@code true} when the trajectory is being followed.
     * The second argument is a pose that should be the trajectory's current target. Ignored if the first argument is {@code false}.
     */
    public BiConsumer<Boolean, Pose2d> addTrajectory(Pose2d[] trajectory) {
        double[] serialized = pose2d(trajectory);
        BiConsumer<Boolean, Pose2d> consumer = (following, target) -> {
            if (following) {
                this.trajectory = serialized;
                trajectoryTarget = pose2d(target);
            } else if (this.trajectory == serialized) {
                this.trajectory = EMPTY_DOUBLE;
                trajectoryTarget = EMPTY_DOUBLE;
            }
        };
        return consumer;
    }

    /**
     * Updates visualizations for vision measurements.
     * @param measurements Estimated poses returned from vision measurements.
     * @param targets Vision targets.
     */
    public void updateVision(Pose2d[] measurements, Pose3d[] targets) {
        visionMeasurements = pose2d(measurements);
        visionTargets = pose3d(targets);
    }

    /**
     * Disables the target visualization.
     */
    public void removeTarget() {
        target = () -> EMPTY_DOUBLE;
    }

    /**
     * Updates the target visualization.
     * @param pose A target {@link Pose2d}.
     */
    public void updateTarget(Pose2d pose) {
        double[] array = pose2d(pose);
        target = () -> array;
    }

    /**
     * Updates the target visualization.
     * @param rotation A target {@link Rotation2d} for the robot to face.
     */
    public void updateTarget(Rotation2d rotation) {
        target =
            () -> {
                Pose2d pose = robot.get();
                return new double[] { pose.getX(), pose.getY(), rotation.getRadians() };
            };
    }

    /**
     * Updates the target visualization.
     * @param rotation A target rotation in radians (CCW positive) for the robot to face.
     */
    public void updateTarget(double rotation) {
        target =
            () -> {
                Pose2d pose = robot.get();
                return new double[] { pose.getX(), pose.getY(), rotation };
            };
    }

    /**
     * Converts module states into a double array.
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
    public static double[] moduleStates(SwerveModuleState[] states) {
        double[] array = new double[states.length * 2];
        for (int i = 0; i < states.length; i++) {
            array[i * 2] = states[i].angle.getRadians();
            array[(i * 2) + 1] = states[i].speedMetersPerSecond;
        }
        return array;
    }

    /**
     * Converts {@link Pose2d}s into a double array.
     * Returns in the following format (heading is in radians):
     * <pre>
     * [
     *   x, y, rot,
     *   x, y, rot,
     *   ...
     * ]
     * </pre>
     */
    public static double[] pose2d(Pose2d... poses) {
        double[] array = new double[poses.length * 3];
        for (int i = 0; i < poses.length; i++) {
            array[i * 3] = poses[i].getX();
            array[(i * 3) + 1] = poses[i].getY();
            array[(i * 3) + 2] = poses[i].getRotation().getRadians();
        }
        return array;
    }

    /**
     * Converts {@link Pose3d}s into a double array.
     * Returns in the following format (rotation is in radians):
     * <pre>
     * [
     *   x, y, z, wRot, xRot, yRot, zRot,
     *   x, y, z, wRot, xRot, yRot, zRot,
     *   ...
     * ]
     * </pre>
     */
    public static double[] pose3d(Pose3d... poses) {
        double[] array = new double[poses.length * 7];
        for (int i = 0; i < poses.length; i++) {
            array[i * 7] = poses[i].getX();
            array[(i * 7) + 1] = poses[i].getY();
            array[(i * 7) + 2] = poses[i].getZ();
            array[(i * 7) + 3] = poses[i].getRotation().getQuaternion().getW();
            array[(i * 7) + 4] = poses[i].getRotation().getQuaternion().getX();
            array[(i * 7) + 5] = poses[i].getRotation().getQuaternion().getY();
            array[(i * 7) + 6] = poses[i].getRotation().getQuaternion().getZ();
        }
        return array;
    }
}
