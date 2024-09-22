package org.team340.lib.logging;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

    public CANcoderLogger() {
        super(CANcoder.class);
    }

    @Override
    public void update(DataLogger logger, CANcoder canCoder) {
        logger.log("absolutePosition", canCoder.getAbsolutePosition().getValue());
        logger.log("magnetHealth", canCoder.getMagnetHealth().getValue().name());
        logger.log("position", canCoder.getPosition().getValue());
        logger.log("velocity", canCoder.getVelocity().getValue());
    }
}
