package org.team340.lib.logging;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(DataLogger logger, TalonFX talonFX) {
        logger.log("acceleration", talonFX.getAcceleration().getValue());
        logger.log("closedLoopError", talonFX.getClosedLoopError().getValue());
        logger.log("deviceTemp", talonFX.getDeviceTemp().getValue());
        logger.log("motorVoltage", talonFX.getMotorVoltage().getValue());
        logger.log("position", talonFX.getPosition().getValue());
        logger.log("statorCurrent", talonFX.getStatorCurrent().getValue());
        logger.log("supplyCurrent", talonFX.getSupplyCurrent().getValue());
        logger.log("supplyVoltage", talonFX.getSupplyVoltage().getValue());
        logger.log("velocity", talonFX.getVelocity().getValue());
    }
}
