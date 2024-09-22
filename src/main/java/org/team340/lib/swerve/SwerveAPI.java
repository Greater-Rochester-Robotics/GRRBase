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
import org.team340.lib.util.Sleep;

public class SwerveAPI implements AutoCloseable {

    public static final class SwerveState {

        public SwerveModulePosition[] modulePositions;
        public SwerveModuleState[] moduleStates;
        public SwerveModuleState[] targetStates;
        public Rotation2d yaw;
        public Rotation2d pitch;
        public Rotation2d roll;
        public Pose2d pose;
        public ChassisSpeeds speeds;

        private SwerveState() {}

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

    private final class SwerveOdometryThread implements AutoCloseable {

        public final AtomicInteger readErrors = new AtomicInteger(0);

        private final BaseStatusSignal[] signals;
        private final boolean timesync;
        private final Thread thread;

        private boolean prioritySet = false;

        public SwerveOdometryThread() {
            List<BaseStatusSignal> signalList = new ArrayList<>();
            signalList.addAll(imu.getSignals());
            for (var module : modules) signalList.addAll(module.getSignals());
            signals = signalList.stream().toArray(BaseStatusSignal[]::new);

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

        public void run(boolean sync) {
            if (sync && thread != null) {
                // No-op
                DriverStation.reportWarning("SwerveAPI.refresh() invoked with asynchronous refreshing enabled", false);
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

            boolean readError = false;
            for (var module : modules) {
                if (!module.refresh()) readError = true;
            }

            if (!phoenixStatus.isOK()) readError = true;
            if (readError) {
                readErrors.incrementAndGet();
                return;
            }

            try {
                stateMutex.writeLock().lock();
                state = new SwerveState();

                state.modulePositions = new SwerveModulePosition[moduleCount];
                state.moduleStates = new SwerveModuleState[moduleCount];
                state.targetStates = targetStates;
                for (int i = 0; i < modules.length; i++) {
                    state.modulePositions[i] = modules[i].getPosition();
                    state.moduleStates[i] = modules[i].getState();
                }

                state.yaw = imu.getYaw();
                state.pitch = imu.getPitch();
                state.roll = imu.getRoll();

                poseEstimator.update(state.yaw, state.modulePositions);
                state.pose = poseEstimator.getEstimatedPosition();
                state.speeds = kinematics.toChassisSpeeds(state.moduleStates);
            } finally {
                stateMutex.writeLock().unlock();
            }
        }

        @Override
        public void close() {
            if (thread != null) {
                try {
                    thread.interrupt();
                    stateMutex.writeLock().unlock();
                } catch (Exception e) {}
            }
        }
    }

    private final int moduleCount;
    private final SwerveConfig config;
    private final SwerveModule[] modules;
    private final Translation2d[] moduleLocations;
    private final SwerveModuleState[] targetStates;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    final SwerveIMU imu;

    private final SwerveOdometryThread odometryThread;
    private final ReadWriteLock stateMutex = new ReentrantReadWriteLock();
    private SwerveState state;

    public SwerveAPI(SwerveConfig config) {
        moduleCount = config.getModules().length;
        this.config = config;

        modules = new SwerveModule[moduleCount];
        moduleLocations = new Translation2d[moduleCount];
        targetStates = new SwerveModuleState[moduleCount];
        for (int i = 0; i < moduleCount; i++) {
            var moduleConfig = config.getModules()[i];
            modules[i] = new SwerveModule(config, moduleConfig);
            moduleLocations[i] = moduleConfig.getLocation();
            targetStates[i] = modules[i].getTargetState();
        }

        state = new SwerveState(moduleCount, targetStates);

        kinematics = new SwerveDriveKinematics(moduleLocations);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.kZero, state.modulePositions, Pose2d.kZero);
        imu = config.getIMU().apply(config);

        odometryThread = new SwerveOdometryThread();
    }

    /**
     * Refreshes inputs from all swerve hardware. This must be called periodically
     * in order for the API to function, <i>only if</i> the odometry period was
     * configured the same as the robot's main loop period, as otherwise inputs
     * are automatically refreshed asynchronously. It is recommended that vision
     * measurements are applied before calling this method.
     */
    public void refresh() {
        odometryThread.run(true);
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
        try {
            stateMutex.readLock().lock();
            return state;
        } finally {
            stateMutex.readLock().unlock();
        }
    }

    /**
     * Gets the robot's swerve modules.
     */
    public SwerveModule[] getModules() {
        return modules;
    }

    @Override
    public void close() {
        try {
            odometryThread.close();
            for (var module : modules) module.close();
            imu.close();
        } catch (Exception e) {}
    }
}
