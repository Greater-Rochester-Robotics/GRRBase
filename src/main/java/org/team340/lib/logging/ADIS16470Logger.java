package org.team340.lib.logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

@CustomLoggerFor(ADIS16470_IMU.class)
public class ADIS16470Logger extends ClassSpecificLogger<ADIS16470_IMU> {

    public ADIS16470Logger() {
        super(ADIS16470_IMU.class);
    }

    @Override
    public void update(DataLogger logger, ADIS16470_IMU adis16470) {
        logger.log("accelerationX", adis16470.getAccelX());
        logger.log("accelerationY", adis16470.getAccelY());
        logger.log("accelerationZ", adis16470.getAccelZ());
        logger.log("velocityX", adis16470.getRate(IMUAxis.kX));
        logger.log("velocityY", adis16470.getRate(IMUAxis.kY));
        logger.log("velocityZ", adis16470.getRate(IMUAxis.kZ));
        logger.log("yaw", adis16470.getAngle(IMUAxis.kYaw));
        logger.log("pitch", adis16470.getAngle(IMUAxis.kPitch));
        logger.log("roll", adis16470.getAngle(IMUAxis.kRoll));
    }
}
