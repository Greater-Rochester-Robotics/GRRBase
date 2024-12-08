package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(CANBus.class)
public class CANBusLogger extends ClassSpecificLogger<CANBus> {

    public CANBusLogger() {
        super(CANBus.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANBus canBus) {
        CANBusStatus status = canBus.getStatus();
        backend.log("isNetworkFD", canBus.isNetworkFD());
        backend.log("busOffCount", status.BusOffCount);
        backend.log("busUtilization", status.BusUtilization);
        backend.log("REC", status.REC);
        backend.log("TEC", status.TEC);
        backend.log("txFullCount", status.TxFullCount);
    }
}
