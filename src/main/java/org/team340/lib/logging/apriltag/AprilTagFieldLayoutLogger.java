package org.team340.lib.logging.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(AprilTagFieldLayout.class)
public class AprilTagFieldLayoutLogger extends ClassSpecificLogger<AprilTagFieldLayout> {

    public AprilTagFieldLayoutLogger() {
        super(AprilTagFieldLayout.class);
    }

    @Override
    public void update(DataLogger logger, AprilTagFieldLayout fieldLayout) {
        // No-op
    }
}
