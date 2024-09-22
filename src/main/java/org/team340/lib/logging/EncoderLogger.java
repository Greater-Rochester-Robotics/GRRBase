package org.team340.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.wpilibj.Encoder;

@CustomLoggerFor(Encoder.class)
public class EncoderLogger extends ClassSpecificLogger<Encoder> {

    public EncoderLogger() {
        super(Encoder.class);
    }

    @Override
    public void update(DataLogger logger, Encoder encoder) {
        logger.log("distance", encoder.getDistance());
        logger.log("rate", encoder.getRate());
    }
}
