package org.team340.lib.logging;

import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(SparkAbsoluteEncoder.class)
public class SparkAbsoluteEncoderLogger extends ClassSpecificLogger<SparkAbsoluteEncoder> {

    public SparkAbsoluteEncoderLogger() {
        super(SparkAbsoluteEncoder.class);
    }

    @Override
    public void update(DataLogger logger, SparkAbsoluteEncoder absoluteEncoder) {
        logger.log("position", absoluteEncoder.getPosition());
        logger.log("velocity", absoluteEncoder.getVelocity());
    }
}
