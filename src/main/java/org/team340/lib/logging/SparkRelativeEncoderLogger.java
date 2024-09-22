package org.team340.lib.logging;

import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(SparkRelativeEncoder.class)
public class SparkRelativeEncoderLogger extends ClassSpecificLogger<SparkRelativeEncoder> {

    public SparkRelativeEncoderLogger() {
        super(SparkRelativeEncoder.class);
    }

    @Override
    public void update(DataLogger logger, SparkRelativeEncoder relativeEncoder) {
        logger.log("position", relativeEncoder.getPosition());
        logger.log("velocity", relativeEncoder.getVelocity());
    }
}
