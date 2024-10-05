package org.team340.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Represents the state of the robot's drivetrain.
 */
public final class SwerveState {

    /**
     * Contains information about the swerve module states and positions.
     */
    public static final class Modules {

        /** The current measured module positions. */
        public final SwerveModulePosition[] positions;
        /** The current measured module states. */
        public final SwerveModuleState[] states;
        /** The next target states of the modules. */
        public final SwerveModuleState[] nextTarget;
        /** The last target states of the modules. */
        public final SwerveModuleState[] lastTarget;

        public Modules(int moduleCount, SwerveModuleState[] nextTarget, SwerveModuleState[] lastTarget) {
            positions = new SwerveModulePosition[moduleCount];
            states = new SwerveModuleState[moduleCount];
            for (int i = 0; i < moduleCount; i++) {
                positions[i] = new SwerveModulePosition();
                states[i] = new SwerveModuleState();
            }

            this.nextTarget = nextTarget;
            this.lastTarget = lastTarget;
        }
    }

    /**
     * Represents the state of the odometry.
     */
    public static final class Odometry {

        /** If the odometry thread is running asynchronously. */
        public boolean async;
        /** If Phoenix timesync is being utilized. */
        public boolean timesync;
        /** The number of successful odometry measurements since the last loop. */
        public int successes;
        /** The number of failing odometry measurements since the last loop. */
        public int failures;

        private Odometry() {}
    }

    /** Information about module states and positions. */
    public final Modules modules;
    /** The state of the odometry. */
    public final Odometry odometry;
    /** The robot's pitch as reported by the IMU. */
    public Rotation2d pitch;
    /** The robot's roll as reported by the IMU. */
    public Rotation2d roll;
    /** The current blue origin relative pose of the robot. */
    public Pose2d pose;
    /** The current measured robot-relative speeds. */
    public ChassisSpeeds speeds;
    /** The directionless measured velocity of the robot. */
    public double velocity;

    SwerveState(int moduleCount, SwerveModuleState[] nextTarget, SwerveModuleState[] lastTarget) {
        modules = new Modules(moduleCount, nextTarget, lastTarget);
        odometry = new Odometry();
        pitch = Rotation2d.kZero;
        roll = Rotation2d.kZero;
        pose = Pose2d.kZero;
        speeds = new ChassisSpeeds();
    }
}
