package org.team340.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.wpilibj.DigitalInput;

@CustomLoggerFor(DigitalInput.class)
public class DigitalInputLogger extends ClassSpecificLogger<DigitalInput> {

    public DigitalInputLogger() {
        super(DigitalInput.class);
    }

    @Override
    public void update(DataLogger logger, DigitalInput digitalInput) {
        logger.log("value", digitalInput.get());
    }
}
