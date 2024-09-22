package org.team340.lib.logging;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(Pigeon2.class)
public class Pigeon2Logger extends ClassSpecificLogger<Pigeon2> {

    public Pigeon2Logger() {
        super(Pigeon2.class);
    }

    @Override
    public void update(DataLogger logger, Pigeon2 pigeon2) {
        logger.log("accelerationX", pigeon2.getAccelerationX().getValue());
        logger.log("accelerationY", pigeon2.getAccelerationY().getValue());
        logger.log("accelerationZ", pigeon2.getAccelerationZ().getValue());
        logger.log("velocityX", pigeon2.getAngularVelocityXWorld().getValue());
        logger.log("velocityY", pigeon2.getAngularVelocityYWorld().getValue());
        logger.log("velocityZ", pigeon2.getAngularVelocityZWorld().getValue());
        logger.log("yaw", pigeon2.getYaw().getValue());
        logger.log("pitch", pigeon2.getPitch().getValue());
        logger.log("roll", pigeon2.getRoll().getValue());
    }
}
