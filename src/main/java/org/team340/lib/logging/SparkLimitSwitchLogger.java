package org.team340.lib.logging;

import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(SparkLimitSwitch.class)
public class SparkLimitSwitchLogger extends ClassSpecificLogger<SparkLimitSwitch> {

    public SparkLimitSwitchLogger() {
        super(SparkLimitSwitch.class);
    }

    @Override
    public void update(DataLogger logger, SparkLimitSwitch limitSwitch) {
        logger.log("isPressed", limitSwitch.isPressed());
    }
}
