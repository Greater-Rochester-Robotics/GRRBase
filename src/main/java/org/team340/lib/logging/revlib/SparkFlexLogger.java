package org.team340.lib.logging.revlib;

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
        double appliedOutput = sparkFlex.getAppliedOutput();
        double busVoltage = sparkFlex.getBusVoltage();

        logger.log("appliedOutput", appliedOutput);
        logger.log("appliedVoltage", appliedOutput * busVoltage);
        logger.log("busVoltage", busVoltage);
        logger.log("motorTemperature", sparkFlex.getMotorTemperature());
        logger.log("outputCurrent", sparkFlex.getOutputCurrent());
        logger.log("position", sparkFlex.getEncoder().getPosition());
        logger.log("velocity", sparkFlex.getEncoder().getVelocity());
    }
}
