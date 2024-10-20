package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(CANBus.class)
public class CANBusLogger extends ClassSpecificLogger<CANBus> {

    public CANBusLogger() {
        super(CANBus.class);
    }

    @Override
    public void update(DataLogger logger, CANBus canBus) {
        CANBusStatus status = canBus.getStatus();
        logger.log("isNetworkFD", canBus.isNetworkFD());
        logger.log("busOffCount", status.BusOffCount);
        logger.log("busUtilization", status.BusUtilization);
        logger.log("REC", status.REC);
        logger.log("TEC", status.TEC);
        logger.log("txFullCount", status.TxFullCount);
    }
}
