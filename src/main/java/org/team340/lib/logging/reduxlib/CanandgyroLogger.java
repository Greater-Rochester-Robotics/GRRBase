package org.team340.lib.logging.reduxlib;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(Canandgyro.class)
public class CanandgyroLogger extends ClassSpecificLogger<Canandgyro> {

    public CanandgyroLogger() {
        super(Canandgyro.class);
    }

    @Override
    public void update(DataLogger logger, Canandgyro canandgyro) {
        logger.log("connected", canandgyro.isConnected());
        logger.log("temperature", canandgyro.getTemperature());
        logger.log("accelerationX", canandgyro.getAccelerationX());
        logger.log("accelerationY", canandgyro.getAccelerationY());
        logger.log("accelerationZ", canandgyro.getAccelerationZ());
        logger.log("velocityX", canandgyro.getAngularVelocityRoll());
        logger.log("velocityY", canandgyro.getAngularVelocityPitch());
        logger.log("velocityZ", canandgyro.getAngularVelocityYaw());
        logger.log("yaw", canandgyro.getYaw());
        logger.log("pitch", canandgyro.getPitch());
        logger.log("roll", canandgyro.getRoll());
    }
}
