package org.team340.lib.blacklight;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Scanner;

/**
 * Helper class for interfacing with Blacklights.
 */
public class Blacklight {

    /**
     * The Blacklight's config.
     */
    private final BlacklightConfig config;
    /**
     * The network table used by the Blacklight.
     */
    private final NetworkTable nt;
    /**
     * Standard deviations for vision measurements.
     */
    private final double[] std;

    private DoubleSubscriber fpsSub;
    private DoubleArraySubscriber poseEstimationSub;
    private DoubleArraySubscriber debugPoseEstimationSub;

    /**
     * Create the Blacklight helper.
     * @param config The Blacklight's config.
     */
    public Blacklight(BlacklightConfig config) {
        config.verify();
        this.config = config;
        std = config.getStandardDeviations();
        nt = NetworkTableInstance.getDefault().getTable("/Blacklight-" + config.getName());
    }

    /**
     * Publishes the Blacklight's configuration to network tables.
     */
    public void publishConfig() {
        String tagLayout = "";
        try {
            Scanner layoutScanner = new Scanner(
                new File(Filesystem.getDeployDirectory(), "blacklight/" + config.getTagLayoutFileName() + ".json")
            );
            while (layoutScanner.hasNextLine()) tagLayout += layoutScanner.nextLine();
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), true);
        }

        NetworkTable configTable = nt.getSubTable("config");
        configTable.getStringTopic("devicePath").publish().set(config.getDevicePath());
        configTable.getIntegerTopic("height").publish().set(config.getHeight());
        configTable.getIntegerTopic("width").publish().set(config.getWidth());
        configTable.getIntegerTopic("autoExposure").publish().set(config.getAutoExposure());
        configTable.getIntegerTopic("absoluteExposure").publish().set(config.getAbsoluteExposure());
        configTable.getIntegerTopic("gain").publish().set(config.getGain());
        configTable.getDoubleArrayTopic("cameraPosition").publish().set(config.getCameraPosition());
        configTable.getDoubleTopic("errorAmbiguity").publish().set(config.getErrorAmbiguity());
        configTable.getDoubleTopic("tagSize").publish().set(config.getTagSize());
        configTable.getStringTopic("tagFamily").publish().set(config.getTagFamily());
        configTable.getStringTopic("tagLayout").publish().set(tagLayout.length() == 0 ? "[]" : tagLayout);
        configTable.getIntegerTopic("debugTag").publish().set(config.getDebugTag());
        configTable.getDoubleArrayTopic("fieldSize").publish().set(config.getFieldSize());
        configTable.getDoubleArrayTopic("fieldMargin").publish().set(config.getFieldMargin());
    }

    /**
     * Start NT listeners for the Blacklight's outputs.
     */
    public void startListeners() {
        NetworkTable outputTable = nt.getSubTable("output");

        if (fpsSub == null) {
            fpsSub = outputTable.getDoubleTopic("fps").subscribe(0);
        }

        if (poseEstimationSub == null) {
            poseEstimationSub =
                outputTable
                    .getDoubleArrayTopic("poseEstimation")
                    .subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        }

        if (debugPoseEstimationSub == null) {
            debugPoseEstimationSub =
                outputTable
                    .getDoubleArrayTopic("debugPoseEstimation")
                    .subscribe(new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
        }
    }

    /**
     * Updates local data from the NT subscriptions. Should be ran periodically.
     * @param poseEstimator The pose estimator in use to be updated.
     */
    public void update(SwerveDrivePoseEstimator poseEstimator) {
        TimestampedDoubleArray[] poseEstimations = poseEstimationSub.readQueue();
        for (int i = 0; i < poseEstimations.length; i++) {
            if (poseEstimations[i].value.length > 0) {
                Pose3d pose = new Pose3d(
                    poseEstimations[i].value[0],
                    poseEstimations[i].value[1],
                    poseEstimations[i].value[2],
                    new Rotation3d(poseEstimations[i].value[3], poseEstimations[i].value[4], poseEstimations[i].value[5])
                );

                double distance = poseEstimations[i].value[6];
                double distanceScale = distance * distance;

                poseEstimator.addVisionMeasurement(
                    pose.toPose2d(),
                    poseEstimations[i].timestamp / 1000000.0,
                    VecBuilder.fill(std[0] * distanceScale, std[1] * distanceScale, std[2] * distanceScale)
                );
            }
        }
    }
}
