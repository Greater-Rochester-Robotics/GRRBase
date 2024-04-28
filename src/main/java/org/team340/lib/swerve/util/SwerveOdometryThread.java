package org.team340.lib.swerve.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Notifier;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Iterator;
import java.util.concurrent.locks.ReentrantLock;
import org.team340.lib.swerve.SwerveModule;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;

/**
 * Samples swerve hardware on a separate thread running at a higher frequency than the main robot loop to provide more accurate odometry.
 * Call {@link SwerveOdometryThread#update(SwerveDrivePoseEstimator)} to update the pose estimator with sampled measurements.
 */
public class SwerveOdometryThread {

    private class Sample {

        public double timestamp;
        public Rotation2d yaw;
        public SwerveModulePosition[] modulePositions;
    }

    private final SwerveModule[] modules;
    private final SwerveIMU imu;

    private final Deque<Sample> samples = new ArrayDeque<>();

    private final Notifier thread;
    private final double period;
    private final int moduleCount;
    private final ReentrantLock mutex = new ReentrantLock();

    private int readErrors = 0;

    /**
     * Creates the odometry thread.
     * @param modules The robot's swerve modules.
     * @param imu The IMU.
     * @param config The general swerve config.
     */
    public SwerveOdometryThread(SwerveModule[] modules, SwerveIMU imu, SwerveConfig config) {
        this.modules = modules;
        this.imu = imu;

        period = config.getOdometryPeriod();
        moduleCount = modules.length;
        if (period != config.getPeriod()) {
            thread = new Notifier(this::recordSample);
            thread.setName("Swerve Odometry");
        } else {
            thread = null;
        }
    }

    /**
     * Starts the thread.
     */
    public void start() {
        if (thread != null) {
            thread.startPeriodic(period);
        }
    }

    /**
     * Updates the pose estimator with recorded odometry samples.
     * @param poseEstimator The pose estimator to update.
     */
    public void update(SwerveDrivePoseEstimator poseEstimator) {
        if (thread == null) recordSample();

        try {
            mutex.lock();
            Iterator<Sample> sampleIterator = samples.iterator();
            while (sampleIterator.hasNext()) {
                Sample sample = sampleIterator.next();
                poseEstimator.updateWithTime(sample.timestamp, sample.yaw, sample.modulePositions);
            }
        } finally {
            samples.clear();
            mutex.unlock();
        }
    }

    /**
     * Returns the number of hardware read errors encountered by the thread.
     */
    public int readErrorCount() {
        return readErrors;
    }

    /**
     * Records a sample.
     */
    private void recordSample() {
        try {
            mutex.lock();
            Sample sample = new Sample();
            int sampleErrors = 0;

            sample.yaw = imu.getYaw();
            if (imu.readError()) sampleErrors++;

            sample.modulePositions = new SwerveModulePosition[moduleCount];
            for (int i = 0; i < moduleCount; i++) {
                sample.modulePositions[i] = modules[i].getModulePosition();
                sampleErrors += modules[i].readErrorCount();
            }

            readErrors += sampleErrors;
            if (sampleErrors == 0) {
                sample.timestamp = MathSharedStore.getTimestamp();
                samples.add(sample);
            }
        } finally {
            mutex.unlock();
        }
    }
}
