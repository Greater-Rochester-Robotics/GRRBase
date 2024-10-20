package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
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
        var acceleration = talonFX.getAcceleration(false);
        var closedLoopError = talonFX.getClosedLoopError(false);
        var deviceTemp = talonFX.getDeviceTemp(false);
        var motorVoltage = talonFX.getMotorVoltage(false);
        var position = talonFX.getPosition(false);
        var statorCurrent = talonFX.getStatorCurrent(false);
        var supplyCurrent = talonFX.getSupplyCurrent(false);
        var supplyVoltage = talonFX.getSupplyVoltage(false);
        var velocity = talonFX.getVelocity(false);

        BaseStatusSignal.refreshAll(
            acceleration,
            closedLoopError,
            deviceTemp,
            motorVoltage,
            position,
            statorCurrent,
            supplyCurrent,
            supplyVoltage,
            velocity
        );

        logger.log("acceleration", acceleration.getValueAsDouble());
        logger.log("closedLoopError", closedLoopError.getValueAsDouble());
        logger.log("deviceTemp", deviceTemp.getValueAsDouble());
        logger.log("motorVoltage", motorVoltage.getValueAsDouble());
        logger.log("position", position.getValueAsDouble());
        logger.log("statorCurrent", statorCurrent.getValueAsDouble());
        logger.log("supplyCurrent", supplyCurrent.getValueAsDouble());
        logger.log("supplyVoltage", supplyVoltage.getValueAsDouble());
        logger.log("velocity", velocity.getValueAsDouble());
    }
}
