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
import java.util.function.Supplier;
import org.team340.lib.swerve.SwerveModule;
import org.team340.lib.swerve.config.SwerveConfig;

/**
 * Samples swerve hardware on a separate thread running at a higher frequency than the main robot loop to provide more accurate odometry.
 * Call {@link SwerveOdometryThread#update(SwerveDrivePoseEstimator)} to update the pose estimator with sampled measurements.
 */
public class SwerveOdometryThread {

    private record ModuleQueues(Deque<Double> distance, Deque<Double> heading) {}

    private final SwerveModule[] modules;
    private final Supplier<Rotation2d> yawSupplier;

    private final ModuleQueues[] moduleQueues;
    private final Deque<Rotation2d> yawQueue = new ArrayDeque<>();
    private final Deque<Double> timestampQueue = new ArrayDeque<>();

    private final Notifier thread;
    private final double period;
    private final ReentrantLock mutex = new ReentrantLock();

    /**
     * Creates the odometry thread.
     * @param modules The robot's swerve modules.
     * @param yawSupplier A supplier for the robot's yaw.
     */
    public SwerveOdometryThread(SwerveModule[] modules, Supplier<Rotation2d> yawSupplier, SwerveConfig config) {
        this.modules = modules;
        this.yawSupplier = yawSupplier;

        moduleQueues = new ModuleQueues[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleQueues[i] = new ModuleQueues(new ArrayDeque<>(), new ArrayDeque<>());
        }

        thread = new Notifier(this::sample);
        thread.setName("Swerve Odometry");
        period = config.getOdometryPeriod();
    }

    /**
     * Starts the thread.
     */
    public void start() {
        thread.startPeriodic(period);
    }

    /**
     * Updates the pose estimator with recorded odometry samples.
     * @param poseEstimator The pose estimator to update.
     */
    public void update(SwerveDrivePoseEstimator poseEstimator) {
        try {
            mutex.lock();
            Iterator<Double> timestampIterator = timestampQueue.iterator();
            while (timestampIterator.hasNext()) {
                double timestamp = timestampIterator.next();

                boolean missingModule = false;
                SwerveModulePosition[] modulePositions = new SwerveModulePosition[moduleQueues.length];
                for (int i = 0; i < modulePositions.length; i++) {
                    double distance, heading;
                    try {
                        distance = moduleQueues[i].distance().pop();
                        heading = moduleQueues[i].heading().pop();
                    } catch (Exception e) {
                        missingModule = true;
                        continue;
                    }

                    modulePositions[i] = new SwerveModulePosition(distance, new Rotation2d(heading));
                }

                if (missingModule) continue;

                try {
                    Rotation2d yaw = yawQueue.pop();
                    poseEstimator.updateWithTime(timestamp, yaw, modulePositions);
                } catch (Exception e) {
                    continue;
                }
            }
        } finally {
            yawQueue.clear();
            timestampQueue.clear();
            for (ModuleQueues queues : moduleQueues) {
                queues.distance().clear();
                queues.heading().clear();
            }

            mutex.unlock();
        }
    }

    /**
     * Samples hardware.
     */
    private void sample() {
        try {
            mutex.lock();
            timestampQueue.add(MathSharedStore.getTimestamp());
            yawQueue.add(yawSupplier.get());
            for (int i = 0; i < modules.length; i++) {
                SwerveModule module = modules[i];
                ModuleQueues queues = moduleQueues[i];
                queues.distance().add(module.getDistance());
                queues.heading().add(module.getHeading());
            }
        } finally {
            mutex.unlock();
        }
    }
}
