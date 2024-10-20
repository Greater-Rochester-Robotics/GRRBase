package org.team340.lib.logging.revlib;

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
        double appliedOutput = sparkMax.getAppliedOutput();
        double busVoltage = sparkMax.getBusVoltage();

        logger.log("appliedOutput", appliedOutput);
        logger.log("appliedVoltage", appliedOutput * busVoltage);
        logger.log("busVoltage", busVoltage);
        logger.log("motorTemperature", sparkMax.getMotorTemperature());
        logger.log("outputCurrent", sparkMax.getOutputCurrent());
        logger.log("position", sparkMax.getEncoder().getPosition());
        logger.log("velocity", sparkMax.getEncoder().getVelocity());
    }
}
