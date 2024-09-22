package org.team340.lib.logging;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(CANSparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<CANSparkMax> {

    public SparkMaxLogger() {
        super(CANSparkMax.class);
    }

    @Override
    public void update(DataLogger logger, CANSparkMax sparkMax) {
        logger.log("appliedOutput", sparkMax.getAppliedOutput());
        logger.log("busVoltage", sparkMax.getBusVoltage());
        logger.log("motorTemperature", sparkMax.getMotorTemperature());
        logger.log("outputCurrent", sparkMax.getOutputCurrent());
        logger.log("position", sparkMax.getEncoder().getPosition());
        logger.log("velocity", sparkMax.getEncoder().getVelocity());
    }
}
