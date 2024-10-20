package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
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
        var absolutePosition = canCoder.getAbsolutePosition(false);
        var magnetHealth = canCoder.getMagnetHealth(false);
        var position = canCoder.getPosition(false);
        var velocity = canCoder.getVelocity(false);

        BaseStatusSignal.refreshAll(absolutePosition, magnetHealth, position, velocity);

        logger.log("absolutePosition", absolutePosition.getValueAsDouble());
        logger.log("magnetHealth", magnetHealth.getValue().name());
        logger.log("position", position.getValueAsDouble());
        logger.log("velocity", velocity.getValueAsDouble());
    }
}
