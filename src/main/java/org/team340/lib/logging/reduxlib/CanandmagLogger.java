package org.team340.lib.logging.reduxlib;

import com.reduxrobotics.sensors.canandmag.Canandmag;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(Canandmag.class)
public class CanandmagLogger extends ClassSpecificLogger<Canandmag> {

    public CanandmagLogger() {
        super(Canandmag.class);
    }

    @Override
    public void update(DataLogger logger, Canandmag canandmag) {
        logger.log("connected", canandmag.isConnected());
        logger.log("absolutePosition", canandmag.getAbsPosition());
        logger.log("magnetInRange", canandmag.magnetInRange());
        logger.log("position", canandmag.getPosition());
        logger.log("temperature", canandmag.getTemperature());
    }
}
