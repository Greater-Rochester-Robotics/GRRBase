package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(Pigeon2.class)
public class Pigeon2Logger extends ClassSpecificLogger<Pigeon2> {

    public Pigeon2Logger() {
        super(Pigeon2.class);
    }

    @Override
    public void update(DataLogger logger, Pigeon2 pigeon2) {
        var temperature = pigeon2.getTemperature(false);
        var accelerationX = pigeon2.getAccelerationX(false);
        var accelerationY = pigeon2.getAccelerationY(false);
        var accelerationZ = pigeon2.getAccelerationZ(false);
        var velocityX = pigeon2.getAngularVelocityXWorld(false);
        var velocityY = pigeon2.getAngularVelocityYWorld(false);
        var velocityZ = pigeon2.getAngularVelocityZWorld(false);
        var yaw = pigeon2.getYaw(false);
        var pitch = pigeon2.getPitch(false);
        var roll = pigeon2.getRoll(false);

        BaseStatusSignal.refreshAll(
            temperature,
            accelerationX,
            accelerationY,
            accelerationZ,
            velocityX,
            velocityY,
            velocityZ,
            yaw,
            pitch,
            roll
        );

        logger.log("temperature", temperature.getValueAsDouble());
        logger.log("accelerationX", accelerationX.getValueAsDouble());
        logger.log("accelerationY", accelerationY.getValueAsDouble());
        logger.log("accelerationZ", accelerationZ.getValueAsDouble());
        logger.log("velocityX", velocityX.getValueAsDouble());
        logger.log("velocityY", velocityY.getValueAsDouble());
        logger.log("velocityZ", velocityZ.getValueAsDouble());
        logger.log("yaw", yaw.getValueAsDouble());
        logger.log("pitch", pitch.getValueAsDouble());
        logger.log("roll", roll.getValueAsDouble());
    }
}
