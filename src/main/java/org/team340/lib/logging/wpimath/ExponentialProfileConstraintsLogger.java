package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.trajectory.ExponentialProfile;

@CustomLoggerFor(ExponentialProfile.Constraints.class)
public class ExponentialProfileConstraintsLogger extends ClassSpecificLogger<ExponentialProfile.Constraints> {

    public ExponentialProfileConstraintsLogger() {
        super(ExponentialProfile.Constraints.class);
    }

    @Override
    public void update(DataLogger logger, ExponentialProfile.Constraints constraints) {
        // No-op
    }
}
