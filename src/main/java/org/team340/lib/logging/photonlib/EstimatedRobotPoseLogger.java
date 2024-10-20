package org.team340.lib.logging.photonlib;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import org.photonvision.EstimatedRobotPose;

@CustomLoggerFor(EstimatedRobotPose.class)
public class EstimatedRobotPoseLogger extends ClassSpecificLogger<EstimatedRobotPose> {

    public EstimatedRobotPoseLogger() {
        super(EstimatedRobotPose.class);
    }

    @Override
    public void update(DataLogger logger, EstimatedRobotPose estimatedRobotPose) {
        // No-op
    }
}
