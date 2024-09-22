package org.team340.lib.controller;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(Controller.class)
public class ControllerLogger extends ClassSpecificLogger<Controller> {

    public ControllerLogger() {
        super(Controller.class);
    }

    @Override
    public void update(DataLogger logger, Controller controller) {
        // No-op, use DriverStation.startDataLog(DataLog)
    }
}
