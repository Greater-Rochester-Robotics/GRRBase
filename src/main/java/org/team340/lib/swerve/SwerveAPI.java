package org.team340.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.SwerveIMUs.SwerveIMU;
import org.team340.lib.util.Math2;
import org.team340.lib.util.Sleep;

public class SwerveAPI implements AutoCloseable {

    /**
     * Represents the state of the robot's drivetrain.
     */
    public static final class SwerveState {

        /** Current positions of the robot's swerve modules. */
        public SwerveModulePosition[] modulePositions;
        /** Current states of the robot's swerve modules. */
        public SwerveModuleState[] moduleStates;
        /** Target states of the robot's swerve modules. */
        public SwerveModuleState[] targetStates;
        /** The robot's yaw. */
        public Rotation2d yaw;
        /** The robot's pitch. */
        public Rotation2d pitch;
        /** The robot's roll. */
        public Rotation2d roll;
        /** The current pose of the robot. */
        public Pose2d pose;
        /** The current robot-relative speeds. */
        public ChassisSpeeds speeds;

        private SwerveState(int moduleCount, SwerveModuleState[] targetStates) {
            modulePositions = new SwerveModulePosition[moduleCount];
            moduleStates = new SwerveModuleState[moduleCount];
            this.targetStates = targetStates;
            for (int i = 0; i < moduleCount; i++) {
                modulePositions[i] = new SwerveModulePosition();
                moduleStates[i] = new SwerveModuleState();
            }
            yaw = Rotation2d.kZero;
            pitch = Rotation2d.kZero;
            roll = Rotation2d.kZero;
            pose = Pose2d.kZero;
            speeds = new ChassisSpeeds();
        }
    }

    final SwerveIMU imu;
    final SwerveModule[] modules;

    private final SwerveConfig config;
    private final int moduleCount;

    private final SwerveState state;
    private final Translation2d[] moduleLocations;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveRatelimiter ratelimiter;

    private final ReadWriteLock odometryMutex = new ReentrantReadWriteLock();
    private final SwerveOdometryThread odometryThread;

    public SwerveAPI(SwerveConfig config) {
        this.config = config;
        moduleCount = config.getModules().length;

        imu = config.getIMU().apply(config);

        modules = new SwerveModule[moduleCount];
        moduleLocations = new Translation2d[moduleCount];
        var targetStates = new SwerveModuleState[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            var moduleConfig = config.getModules()[i];
            modules[i] = new SwerveModule(config, moduleConfig);
            moduleLocations[i] = moduleConfig.getLocation();
            targetStates[i] = modules[i].getTargetState();
        }

        state = new SwerveState(moduleCount, targetStates);

        kinematics = new SwerveDriveKinematics(moduleLocations);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, state.modulePositions, Pose2d.kZero);
        ratelimiter = new SwerveRatelimiter(config, kinematics);

        odometryThread = new SwerveOdometryThread();
    }

    /**
     * Refreshes inputs from all swerve hardware. This must be called periodically
     * in order for the API to function. Typically, this method is called at the
     * start of the swerve subsystem's {@code periodic()} method.
     */
    public void refresh() {
        if (!odometryThread.active()) odometryThread.run(true);
        try {
            odometryMutex.readLock().lock();
            for (int i = 0; i < moduleCount; i++) {
                var volatilePosition = modules[i].getPosition();
                state.modulePositions[i].distanceMeters = volatilePosition.distanceMeters;
                state.modulePositions[i].angle = volatilePosition.angle;
                var volatileState = modules[i].getState();
                state.moduleStates[i].speedMetersPerSecond = volatileState.speedMetersPerSecond;
                state.moduleStates[i].angle = volatileState.angle;
            }
            state.yaw = imu.getYaw();
            state.pitch = imu.getPitch();
            state.roll = imu.getRoll();
            state.pose = poseEstimator.getEstimatedPosition();
            state.speeds = kinematics.toChassisSpeeds(state.moduleStates);
        } finally {
            odometryMutex.readLock().unlock();
        }
    }

    /**
     * Gets the total number of errors since startup encountered
     * by the odometry thread while reading inputs from devices.
     */
    public int getReadErrors() {
        return odometryThread.readErrors.get();
    }

    /**
     * Gets the current state of the robot's swerve drivetrain.
     */
    public SwerveState getState() {
        return state;
    }

    /**
     * Drives using chassis speeds.
     * @param chassisSpeeds The chassis speeds to apply.
     * @param fieldRelative If the speeds are field-relative.
     * @param discretize If the speeds should be discretized.
     * @param ratelimit If the ratelimiter should be used to calculate module states.
     */
    public void applySpeeds(ChassisSpeeds chassisSpeeds, boolean fieldRelative, boolean discretize, boolean ratelimit) {
        if (discretize) {
            double dtheta = chassisSpeeds.omegaRadiansPerSecond * config.getDiscretizationPeriod();

            double sin = -dtheta / 2.0;
            double cos = Math2.epsilonEquals(Math.cos(dtheta) - 1.0, 0.0)
                ? 1.0 - ((1.0 / 12.0) * dtheta * dtheta)
                : (sin * Math.sin(dtheta)) / (Math.cos(dtheta) - 1.0);

            double dt = config.getPeriod();
            double dx = chassisSpeeds.vxMetersPerSecond * dt;
            double dy = chassisSpeeds.vyMetersPerSecond * dt;

            chassisSpeeds =
                new ChassisSpeeds(((dx * cos) - (dy * sin)) / dt, ((dx * sin) + (dy * cos)) / dt, chassisSpeeds.omegaRadiansPerSecond);
        }

        if (ratelimit) {
            applyStates(ratelimiter.calculate(chassisSpeeds, state));
        } else {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, config.getVelocity());
            applyStates(states);
        }
    }

    /**
     * Drives using module states.
     * @param states The states to apply.
     */
    public void applyStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            var module = modules[i];
            if (module != null) module.applyState(states[i]);
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

    @Override
    public void close() {
        try {
            odometryThread.close();
            for (var module : modules) module.close();
            imu.close();
        } catch (Exception e) {}
    }

    private final class SwerveOdometryThread implements AutoCloseable {

        public final AtomicInteger readErrors = new AtomicInteger(0);

        private final SwerveModulePosition[] positionCache;
        private final BaseStatusSignal[] signals;
        private final boolean timesync;
        private final Thread thread;

        private boolean prioritySet = false;

        public SwerveOdometryThread() {
            List<BaseStatusSignal> signalList = new ArrayList<>();
            signalList.addAll(imu.getSignals());
            for (var module : modules) signalList.addAll(module.getSignals());
            signals = signalList.stream().toArray(BaseStatusSignal[]::new);
            positionCache = new SwerveModulePosition[moduleCount];
            for (int i = 0; i < moduleCount; i++) {
                modules[i].getPosition();
            }

            timesync = config.getPhoenixPro() && CANBus.isNetworkFD(config.getPhoenixCanBus());
            if (config.getOdometryPeriod() < config.getPeriod()) {
                thread = new Thread(() -> this.run(false));
                thread.setName("SwerveAPI");
                thread.setDaemon(true);
                thread.start();
            } else {
                thread = null;
            }
        }

        public boolean active() {
            return thread != null;
        }

        public void run(boolean sync) {
            if (sync && active()) {
                // No-op
                DriverStation.reportWarning("SwerveOdometryThread.run(true) invoked with asynchronous refreshing enabled", false);
                return;
            } else if (!sync && !prioritySet) {
                Threads.setCurrentThreadPriority(true, 1);
            }

            StatusCode phoenixStatus;
            if (timesync && !sync) {
                phoenixStatus = BaseStatusSignal.waitForAll(config.getOdometryPeriod() * 2.0, signals);
            } else {
                if (!sync) Sleep.seconds(config.getOdometryPeriod(), true);
                phoenixStatus = BaseStatusSignal.refreshAll(signals);
            }

            try {
                odometryMutex.writeLock().lock();
                boolean readError = !phoenixStatus.isOK();
                for (var module : modules) {
                    if (!module.refresh()) readError = true;
                }

                if (readError) {
                    readErrors.incrementAndGet();
                    return;
                }

                poseEstimator.update(imu.getYaw(), positionCache);
            } finally {
                odometryMutex.writeLock().unlock();
            }
        }

        @Override
        public void close() {
            if (thread != null) {
                try {
                    thread.interrupt();
                    odometryMutex.writeLock().unlock();
                } catch (Exception e) {}
            }
        }
    }
}
