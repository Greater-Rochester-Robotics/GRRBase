package org.team340.lib.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import org.team340.lib.swerve.SwerveAPI.TimestampedYaw;

/**
 * Represents the state of the robot's drivetrain.
 */
@Logged(strategy = Strategy.OPT_IN)
public final class SwerveState {

    /**
     * Contains information about swerve module states and positions.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class Modules {

        /** The current measured module positions. */
        public final SwerveModulePosition[] positions;
        /** The current measured module states. */
        public final SwerveModuleState[] states;
        /** The last target states of the modules. */
        public final SwerveModuleState[] lastTarget;
        /** The next target states of the modules. */
        public final SwerveModuleState[] nextTarget;

        private Modules(SwerveModule[] modules) {
            positions = new SwerveModulePosition[modules.length];
            states = new SwerveModuleState[modules.length];
            lastTarget = new SwerveModuleState[modules.length];
            nextTarget = new SwerveModuleState[modules.length];
            for (int i = 0; i < modules.length; i++) {
                positions[i] = new SwerveModulePosition();
                states[i] = new SwerveModuleState();
                lastTarget[i] = modules[i].getLastTarget();
                nextTarget[i] = modules[i].getNextTarget();
            }
        }
    }

    /**
     * Measurements from the IMU.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class IMU {

        /** All yaw measurements since the {@link SwerveState} has been refreshed. */
        public final List<TimestampedYaw> yawMeasurements;
        /** The robot's yaw. */
        public Rotation2d yaw;
        /** The robot's pitch. */
        public Rotation2d pitch;
        /** The robot's roll. */
        public Rotation2d roll;

        private IMU() {
            yawMeasurements = new ArrayList<>();
            yaw = Rotation2d.kZero;
            pitch = Rotation2d.kZero;
            roll = Rotation2d.kZero;
        }
    }

    /**
     * Represents the state of the odometry thread.
     */
    @Logged(strategy = Strategy.OPT_IN)
    public static final class OdometryThread {

        /** If Phoenix timesync is being utilized. */
        public boolean timesync;
        /** The number of successful odometry measurements since the last loop. */
        public int successes;
        /** The number of failing odometry measurements since the last loop. */
        public int failures;

        private OdometryThread() {}
    }

    /** Information about module states and positions. */
    public final Modules modules;
    /** Measurements from the IMU. */
    public final IMU imu;
    /** The state of the odometry thread. */
    public final OdometryThread odometryThread;
    /** The current measured robot-relative speeds. */
    public final ChassisSpeeds speeds;
    /** The next target robot-relative speeds. Updated when using {@code applySpeeds()}. */
    public final ChassisSpeeds targetSpeeds;
    /** The directionless measured velocity of the robot. */
    public double velocity;
    /** The current blue origin relative pose of the robot. */
    public Pose2d pose;
    /** The current blue origin relative translation of the robot. */
    public Translation2d translation;
    /** The robot's rotation (yaw) as reported from the pose estimator. */
    public Rotation2d rotation;
    /** The timestamp of the swerve state in seconds (FPGA time). */
    public double timestamp;

    SwerveState(SwerveModule[] modules) {
        this.modules = new Modules(modules);
        imu = new IMU();
        odometryThread = new OdometryThread();
        speeds = new ChassisSpeeds();
        targetSpeeds = new ChassisSpeeds();
        pose = Pose2d.kZero;
        translation = Translation2d.kZero;
        rotation = Rotation2d.kZero;
    }
}
