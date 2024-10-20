package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@CustomLoggerFor(TrapezoidProfile.class)
public class TrapezoidProfileLogger extends ClassSpecificLogger<TrapezoidProfile> {

    public TrapezoidProfileLogger() {
        super(TrapezoidProfile.class);
    }

    @Override
    public void update(DataLogger logger, TrapezoidProfile profile) {
        // No-op
    }
}
