package org.team340.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.SwerveIMUs.SwerveIMU;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Sleep;
import org.team340.robot.Robot;

public class SwerveAPI implements AutoCloseable {

    final SwerveIMU imu;
    final SwerveModule[] modules;
    final SwerveConfig config;

    private final int moduleCount;
    private final double farthestModule;
    private final Translation2d[] moduleLocations;
    private final SwerveModuleState[] lockedStates;

    private final SwerveState state;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private final Lock odometryMutex = new ReentrantLock();
    private final SwerveOdometryThread odometryThread;

    private Rotation2d lastRobotAngle = Math2.kZeroRotation2d;
    private double lastRatelimit = 0.0;

    private Consumer<ChassisSpeeds> imuSimHook = s -> {};

    public SwerveAPI(SwerveConfig config) {
        this.config = config;

        moduleCount = config.modules.length;
        modules = new SwerveModule[moduleCount];
        moduleLocations = new Translation2d[moduleCount];
        lockedStates = new SwerveModuleState[moduleCount];
        double farthest = 0.0;
        for (int i = 0; i < moduleCount; i++) {
            var moduleConfig = config.modules[i];
            modules[i] = new SwerveModule(config, moduleConfig);
            moduleLocations[i] = moduleConfig.location;
            lockedStates[i] = new SwerveModuleState(0.0, moduleLocations[i].getAngle());
            farthest = Math.max(farthest, moduleLocations[i].getNorm());
        }

        farthestModule = farthest;

        state = new SwerveState(modules);
        kinematics = new SwerveDriveKinematics(moduleLocations);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            Math2.kZeroRotation2d,
            state.modules.positions,
            Math2.kZeroPose2d,
            config.odometryStdDevs,
            VecBuilder.fill(0.0, 0.0, 0.0)
        );

        imu = SwerveIMU.construct(config.imu, config, hook -> imuSimHook = hook);
        odometryThread = new SwerveOdometryThread();
    }

    /**
     * Gets the current state of the robot's swerve drivetrain.
     */
    public SwerveState getState() {
        return state;
    }

    /**
     * Refreshes inputs from all swerve hardware. This must be called periodically
     * in order for the API to function. Typically, this method is called at the
     * start of the swerve subsystem's {@code periodic()} method.
     */
    public void refresh() {
        odometryMutex.lock();
        try {
            odometryThread.run(true);
            state.odometry.timesync = odometryThread.timesync;
            state.odometry.successes = odometryThread.successes;
            state.odometry.failures = odometryThread.failures;

            odometryThread.successes = 0;
            odometryThread.failures = 0;

            for (int i = 0; i < moduleCount; i++) {
                Math2.copyInto(modules[i].getPosition(), state.modules.positions[i]);
                Math2.copyInto(modules[i].getState(), state.modules.states[i]);
            }

            state.pose = poseEstimator.getEstimatedPosition();
        } finally {
            odometryMutex.unlock();
        }

        state.pitch = imu.getPitch();
        state.roll = imu.getRoll();
        state.speeds = kinematics.toChassisSpeeds(state.modules.states);
        state.velocity = Math.hypot(state.speeds.vxMetersPerSecond, state.speeds.vyMetersPerSecond);

        imuSimHook.accept(state.speeds);
    }

    /**
     * Tares the rotation of the robot. Useful for fixing an out of sync or drifting IMU. In
     * most cases, a forward perspective of {@link ForwardPerspective#OPERATOR OPERATOR} is
     * desirable. {@link ForwardPerspective#ROBOT ROBOT} will no-op.
     * @param forwardPerspective The perspective to tare the rotation to.
     */
    public void tareRotation(ForwardPerspective forwardPerspective) {
        if (forwardPerspective.equals(ForwardPerspective.ROBOT)) return;
        var rotation = forwardPerspective.getTareRotation();
        resetPose(new Pose2d(state.pose.getTranslation(), rotation));
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * Additionally, odometry and vision measurement history is flushed. This
     * method is typically invoked at the start of a match to set the robot's
     * position to the starting location of an autonomous mode. The supplied
     * pose is expected to be blue origin relative.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public void resetPose(Pose2d pose) {
        odometryMutex.lock();
        try {
            poseEstimator.resetPosition(odometryThread.lastYaw, state.modules.positions, pose);
            state.pose = poseEstimator.getEstimatedPosition();
        } finally {
            odometryMutex.unlock();
        }
    }

    /**
     * Adds a vision measurement to the pose estimator.
     * @see {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp The timestamp of the vision measurement in seconds.
     * @param stdDevs Standard deviations of the vision pose measurement (x position in meters, y position in meters, and yaw in radians).
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        odometryMutex.lock();
        try {
            poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDevs);
            state.pose = poseEstimator.getEstimatedPosition();
        } finally {
            odometryMutex.unlock();
        }
    }

    /**
     * Drives using inputs from the driver's controller. The {@code x} and {@code y} parameters
     * expect the controller's NED (north-east-down) convention, and will automatically convert
     * to WPILib's typical NWU (north-west-up) convention when applying chassis speeds.
     * @param x The X value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param y The Y value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     * @param forwardPerspective The forward perspective for the chassis speeds.
     * @param discretize If the generated speeds should be discretized.
     * @param ratelimit If the robot's acceleration should be constrained.
     */
    public void applyDriverInput(
        double x,
        double y,
        double angular,
        ForwardPerspective forwardPerspective,
        boolean discretize,
        boolean ratelimit
    ) {
        double angularVel =
            config.driverAngularVel * Math.copySign(Math.pow(angular, config.driverAngularVelExp), angular);
        applyDriverXY(x, y, angularVel, forwardPerspective, discretize, ratelimit);
    }

    /**
     * Drives using inputs from the driver's controller, with a specified angular velocity in radians/second.
     * Use this method as opposed to {@link SwerveAPI#applyDriverInput(double, double, double)} if the driver's
     * input is desired only for x/y movement, and not heading. Use cases include locking the robot's heading by
     * passing the output of a PID controller as the angular velocity. The {@code x} and {@code y} parameters
     * expect the controller's NED (north-east-down) convention, and will automatically convert to WPILib's
     * typical NWU (north-west-up) convention when applying chassis speeds.
     * @param x The X value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param y The Y value of the driver's joystick, from {@code [-1.0, 1.0]}.
     * @param angularVel The CCW+ angular velocity to apply, in radians/second.
     * @param forwardPerspective The forward perspective for the chassis speeds.
     * @param discretize If the generated speeds should be discretized.
     * @param ratelimit If the robot's acceleration should be constrained.
     */
    public void applyDriverXY(
        double x,
        double y,
        double angularVel,
        ForwardPerspective forwardPerspective,
        boolean discretize,
        boolean ratelimit
    ) {
        double norm = Math.hypot(x, y);
        if (norm > 1.0) {
            x /= norm;
            y /= norm;
        }
        double xyMult = config.driverVel * Math.pow(Math.hypot(x, y), config.driverVelExp - 1.0);
        applySpeeds(new ChassisSpeeds(-y * xyMult, -x * xyMult, angularVel), forwardPerspective, discretize, ratelimit);
    }

    /**
     * Drives using chassis speeds.
     * @param speeds The chassis speeds to apply. Note that the provided {@link ChassisSpeeds} object may be mutated.
     * @param forwardPerspective The forward perspective for the chassis speeds.
     * @param discretize If the speeds should be discretized.
     * @param ratelimit If the robot's acceleration should be constrained.
     */
    public void applySpeeds(
        ChassisSpeeds speeds,
        ForwardPerspective forwardPerspective,
        boolean discretize,
        boolean ratelimit
    ) {
        double w_max = config.velocity / farthestModule;
        if (Math.abs(speeds.omegaRadiansPerSecond) >= w_max) {
            speeds.vxMetersPerSecond = 0.0;
            speeds.vyMetersPerSecond = 0.0;
            speeds.omegaRadiansPerSecond = Math.copySign(w_max, speeds.omegaRadiansPerSecond);
        } else {
            double vx = speeds.vxMetersPerSecond;
            double vy = speeds.vyMetersPerSecond;
            double w = speeds.omegaRadiansPerSecond;

            double k = 1.0;
            double v_max2 = config.velocity * config.velocity;

            for (var r : moduleLocations) {
                double vx_w = -w * r.getY();
                double vy_w = w * r.getX();

                double vx_m = (vx + vx_w);
                double vy_m = (vy + vy_w);

                if ((vx_m * vx_m + vy_m * vy_m) > v_max2) {
                    double a = vx * vx + vy * vy;
                    double b = 2 * vx * vx_w + 2 * vy * vy_w;
                    double c = vx_w * vx_w + vy_w * vy_w - v_max2;
                    k = Math.min(k, (2 * c) / (-b - Math.sqrt(b * b - 4 * a * c)));
                }
            }

            speeds.vxMetersPerSecond *= k;
            speeds.vyMetersPerSecond *= k;
        }

        if (ratelimit) {
            double now = Timer.getFPGATimestamp();
            if (now - lastRatelimit > config.period * 2.0) {
                Math2.copyInto(state.speeds, state.targetSpeeds);
            }
            lastRatelimit = now;

            forwardPerspective.toPerspectiveSpeeds(state.targetSpeeds, lastRobotAngle);

            double vx_l = state.targetSpeeds.vxMetersPerSecond;
            double vy_l = state.targetSpeeds.vyMetersPerSecond;
            double v_l = Math.hypot(vx_l, vy_l);
            double w_l = state.targetSpeeds.omegaRadiansPerSecond;

            double dx = speeds.vxMetersPerSecond - vx_l;
            double dy = speeds.vyMetersPerSecond - vy_l;
            double a_slip = config.slipAccel * config.period;
            if (dx * dx + dy * dy > a_slip * a_slip) {
                double s = a_slip / Math.hypot(dx, dy);
                speeds.vxMetersPerSecond = vx_l + (s * dx);
                speeds.vyMetersPerSecond = vy_l + (s * dy);
            }

            double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            double a_torque = (config.torqueAccel * config.period) * (1.0 - (v_l / config.velocity));
            if (v - v_l > a_torque) {
                double s = (v_l + a_torque) / v;
                speeds.vxMetersPerSecond *= s;
                speeds.vyMetersPerSecond *= s;
            }

            double dw = speeds.omegaRadiansPerSecond - w_l;
            double a_angular = config.angularAccel * config.period;
            speeds.omegaRadiansPerSecond = w_l + ((Math.abs(dw) > a_angular ? a_angular / Math.abs(dw) : 1.0) * dw);
        }

        if (discretize) ChassisSpeeds.discretize(speeds, config.discretizationPeriod);

        lastRobotAngle = state.pose.getRotation();
        forwardPerspective.toRobotSpeeds(speeds, lastRobotAngle);
        Math2.copyInto(speeds, state.targetSpeeds);

        applyStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Drives the modules to an X formation to stop the robot from moving.
     */
    public void applyLockedWheels() {
        applyStates(lockedStates);
    }

    /**
     * Drives using module states.
     * @param states The states to apply.
     */
    public void applyStates(SwerveModuleState[] states) {
        if (moduleCount != states.length) {
            throw new IllegalArgumentException(
                "Requested " + states.length + " states be applied to " + moduleCount + " modules"
            );
        }

        for (int i = 0; i < moduleCount; i++) {
            modules[i].applyState(states[i]);
        }
    }

    /**
     * Drives the robot using open-loop voltage. Intended for characterization.
     * Plumbing for recording device voltage via their Java API is intentionally
     * unavailable, as GC pressure and CAN latency will result in inaccurate data.
     * Use Phoenix Signal Logging or URCL instead.
     * @param voltage The voltage to apply to the move motors.
     * @param angle The robot-relative angle to apply to the turn motors.
     */
    public void applyVoltage(double voltage, Rotation2d angle) {
        for (int i = 0; i < moduleCount; i++) {
            modules[i].applyVoltage(voltage, angle);
        }
    }

    /**
     * Enables publishing tunables for adjustment of the API's constants.
     * @param name The parent name for the tunables in NetworkTables.
     */
    public void enableTunables(String name) {
        SwerveTunables.initialize(name, this);
    }

    @Override
    public void close() {
        try {
            odometryThread.close();
            for (var module : modules) module.close();
            imu.close();
        } catch (Exception e) {}
    }

    /**
     * Specifies the X+ direction of chassis speeds.
     */
    public static enum ForwardPerspective {
        // TODO Remove copyInto, mutate in place: blocked by upstream PR https://github.com/wpilibsuite/allwpilib/pull/7115

        /**
         * The speeds are relative to the operator's perspective. If the robot
         * is on the blue alliance, X+ drives towards the red alliance. If the
         * robot is on the red alliance, X+ drives towards the blue alliance.
         */
        OPERATOR {
            @Override
            void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
                (Alliance.isBlue() ? BLUE_ALLIANCE : RED_ALLIANCE).toRobotSpeeds(speeds, robotAngle);
            }

            @Override
            void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
                (Alliance.isBlue() ? BLUE_ALLIANCE : RED_ALLIANCE).toPerspectiveSpeeds(speeds, robotAngle);
            }

            @Override
            Rotation2d getTareRotation() {
                return (Alliance.isBlue() ? BLUE_ALLIANCE : RED_ALLIANCE).getTareRotation();
            }
        },

        /**
         * The speeds are relative to the blue alliance perspective.
         * X+ drives towards the red alliance.
         */
        BLUE_ALLIANCE {
            @Override
            void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
                var newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle);
                Math2.copyInto(newSpeeds, speeds);
            }

            @Override
            void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
                var newSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotAngle);
                Math2.copyInto(newSpeeds, speeds);
            }

            @Override
            Rotation2d getTareRotation() {
                return Math2.kZeroRotation2d;
            }
        },

        /**
         * The speeds are relative to the red alliance perspective.
         * X+ drives towards the blue alliance.
         */
        RED_ALLIANCE {
            @Override
            void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
                var newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotAngle.rotateBy(Math2.kPiRotation2d));
                Math2.copyInto(newSpeeds, speeds);
            }

            @Override
            void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
                var newSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotAngle.rotateBy(Math2.kPiRotation2d));
                Math2.copyInto(newSpeeds, speeds);
            }

            @Override
            Rotation2d getTareRotation() {
                return Math2.kPiRotation2d;
            }
        },

        /**
         * The speeds are relative to the robot's perspective.
         * X+ drives forwards relative to the chassis.
         */
        ROBOT {
            @Override
            void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {}

            @Override
            void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {}

            @Override
            Rotation2d getTareRotation() {
                // Will no-op downstream.
                return null;
            }
        };

        /**
         * Converts perspective relative speeds to robot relative speeds.
         * @param speeds The perspective relative speeds to convert.
         * @param robotAngle The blue origin relative angle of the robot.
         */
        abstract void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle);

        /**
         * Converts robot relative speeds to the perspective relative speeds.
         * @param speeds The robot relative speeds to convert.
         * @param robotAngle The blue origin relative angle of the robot.
         */
        abstract void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle);

        /**
         * Gets the rotation to apply as the new zero when
         * taring the robot's rotation to the perspective.
         */
        abstract Rotation2d getTareRotation();
    }

    /**
     * Manages swerve odometry. Will run asynchronously at the configured odometry update
     * period, unless the configured period is the same or more than the main robot loop
     * period. The {@link SwerveOdometryThread#run(boolean)} method is also invoked in
     * {@link SwerveAPI#refresh()}, to ensure the latest measurements are applied to
     * pose estimation when executing user logic in the main loop.
     */
    private final class SwerveOdometryThread implements AutoCloseable {

        public Rotation2d lastYaw = Math2.kZeroRotation2d;
        public boolean timesync = false;
        public int successes = 0;
        public int failures = 0;

        private final SwerveModulePosition[] positionCache;
        private final BaseStatusSignal[] signals;
        private final Thread thread;

        private volatile boolean active = false;
        private double lastTime = 0.0;

        public SwerveOdometryThread() {
            List<BaseStatusSignal> signalList = new ArrayList<>();
            signalList.addAll(imu.getSignals());
            for (var module : modules) signalList.addAll(module.getSignals());
            signals = signalList.stream().toArray(BaseStatusSignal[]::new);
            positionCache = new SwerveModulePosition[moduleCount];
            for (int i = 0; i < moduleCount; i++) {
                positionCache[i] = modules[i].getPosition();
            }

            if (config.odometryPeriod < config.period) {
                thread = new Thread(() -> {
                    Threads.setCurrentThreadPriority(true, 1);
                    while (active) this.run(false);
                });
                thread.setName("SwerveAPI");
                thread.setDaemon(true);
                timesync = config.phoenixPro && CANBus.isNetworkFD(config.phoenixCanBus);
                active = true;
                thread.start();
            } else {
                thread = null;
            }
        }

        /**
         * Runs an odometry update.
         * @param sync If the update is being invoked from the main loop.
         */
        public void run(boolean sync) {
            StatusCode phoenixStatus = StatusCode.OK;
            if (!sync) {
                if (timesync) {
                    phoenixStatus = BaseStatusSignal.waitForAll(config.odometryPeriod * 2.0, signals);
                } else {
                    Sleep.seconds(Math.max(0.0, config.odometryPeriod - (Timer.getFPGATimestamp() - lastTime)));
                    lastTime = Timer.getFPGATimestamp();
                }
            }

            odometryMutex.lock();
            try {
                if (!timesync && signals.length > 0) phoenixStatus = BaseStatusSignal.refreshAll(signals);

                lastYaw = imu.getYaw();

                boolean readError = !phoenixStatus.isOK() || imu.readError();
                for (var module : modules) {
                    if (!module.refresh()) readError = true;
                }

                if (readError) {
                    failures++;
                    if (!Robot.isSimulation()) return;
                }

                poseEstimator.update(lastYaw, positionCache);
                successes++;
            } finally {
                odometryMutex.unlock();
            }
        }

        @Override
        public void close() {
            if (active) {
                active = false;
                if (thread != null && thread.isAlive()) {
                    try {
                        thread.interrupt();
                        thread.join();
                    } catch (Exception e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }
    }
}
