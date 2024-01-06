package org.team340.lib.swerve.hardware.encoders.vendors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;
import org.team340.lib.util.Math2;

/**
 * CTRE CANcoder swerve wrapper.
 */
public class SwerveCANcoder implements SwerveEncoder {

    private final StatusSignal<Double> absolutePositionSignal;

    /**
     * Create the CANcoder wrapper.
     * @param canCoder The CANcoder to wrap.
     * @param config The general swerve config.
     * @param moduleConfig The encoder's module's config.
     */
    public SwerveCANcoder(CANcoder canCoder, SwerveConfig config, SwerveModuleConfig moduleConfig) {
        absolutePositionSignal = canCoder.getAbsolutePosition();

        double hz = 1.0 / config.getPeriod();
        absolutePositionSignal.setUpdateFrequency(hz);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();

        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoderConfig.MagnetSensor.MagnetOffset = moduleConfig.getEncoderOffset() / Math2.TWO_PI;
        canCoderConfig.MagnetSensor.SensorDirection =
            moduleConfig.getEncoderInverted() ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;

        canCoder.getConfigurator().apply(canCoderConfig);
    }

    @Override
    public double getPosition() {
        return absolutePositionSignal.refresh().getValue() * Math2.TWO_PI;
    }
}
