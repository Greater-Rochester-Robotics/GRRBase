package org.team340.lib.logging.wpimath;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@CustomLoggerFor(TrapezoidProfile.State.class)
public class TrapezoidProfileStateLogger extends ClassSpecificLogger<TrapezoidProfile.State> {

    public TrapezoidProfileStateLogger() {
        super(TrapezoidProfile.State.class);
    }

    @Override
    public void update(DataLogger logger, TrapezoidProfile.State state) {
        logger.log("position", state.position);
        logger.log("velocity", state.velocity);
    }
}
