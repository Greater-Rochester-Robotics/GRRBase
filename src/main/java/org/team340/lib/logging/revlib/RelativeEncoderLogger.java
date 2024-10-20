package org.team340.lib.logging.revlib;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {

    public RelativeEncoderLogger() {
        super(RelativeEncoder.class);
    }

    @Override
    public void update(DataLogger logger, RelativeEncoder relativeEncoder) {
        logger.log("position", relativeEncoder.getPosition());
        logger.log("velocity", relativeEncoder.getVelocity());
    }
}
