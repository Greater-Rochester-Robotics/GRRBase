package org.team340.lib.logging.revlib;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(SparkPIDController.class)
public class SparkPIDControllerLogger extends ClassSpecificLogger<SparkPIDController> {

    public SparkPIDControllerLogger() {
        super(SparkPIDController.class);
    }

    @Override
    public void update(DataLogger logger, SparkPIDController pidController) {
        // No-op
    }
}
