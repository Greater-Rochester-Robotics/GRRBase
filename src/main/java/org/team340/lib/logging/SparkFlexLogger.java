package org.team340.lib.logging;

import com.revrobotics.CANSparkFlex;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(CANSparkFlex.class)
public class SparkFlexLogger extends ClassSpecificLogger<CANSparkFlex> {

    public SparkFlexLogger() {
        super(CANSparkFlex.class);
    }

    @Override
    public void update(DataLogger logger, CANSparkFlex sparkFlex) {
        logger.log("appliedOutput", sparkFlex.getAppliedOutput());
        logger.log("busVoltage", sparkFlex.getBusVoltage());
        logger.log("motorTemperature", sparkFlex.getMotorTemperature());
        logger.log("outputCurrent", sparkFlex.getOutputCurrent());
        logger.log("position", sparkFlex.getEncoder().getPosition());
        logger.log("velocity", sparkFlex.getEncoder().getVelocity());
    }
}
