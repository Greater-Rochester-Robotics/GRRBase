package org.team340.lib.swerve.hardware.encoders.vendors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.team340.lib.swerve.SwerveBase.SwerveAbsoluteEncoderType;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveAbsoluteEncoder;
import org.team340.lib.util.Math2;

/**
 * CTRE CANcoder swerve wrapper.
 */
public class SwerveCANcoder implements SwerveAbsoluteEncoder {

    /**
     * The CANcoder.
     */
    private final CANcoder canCoder;

    /**
     * Create the CANcoder wrapper.
     * @param canCoder The CANcoder to wrap.
     * @param config The general swerve config.
     * @param moduleConfig The encoder's module's config.
     */
    public SwerveCANcoder(CANcoder canCoder, SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.canCoder = canCoder;

        double hz = 1.0 / config.getPeriod();
        canCoder.getVelocity().setUpdateFrequency(hz);
        canCoder.getPosition().setUpdateFrequency(hz);
        canCoder.getAbsolutePosition().setUpdateFrequency(hz);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfig.MagnetSensor.MagnetOffset = moduleConfig.getAbsoluteEncoderOffset() / Math2.TWO_PI;
        canCoderConfig.MagnetSensor.SensorDirection =
            moduleConfig.getAbsoluteEncoderInverted()
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        canCoder.getConfigurator().apply(canCoderConfig);
    }

    /**
     * Gets the encoder's unclamped position.
     * Used by {@link SwerveTalonFX} (Talon FX + CANcoder has unique behavior where the CANcoder is used for on-device turn PID instead of the integrated encoder).
     */
    public double getRelativePosition() {
        return canCoder.getPosition().getValue() * Math2.TWO_PI;
    }

    @Override
    public double getAbsolutePosition() {
        return canCoder.getAbsolutePosition().getValue() * Math2.TWO_PI;
    }

    @Override
    public SwerveAbsoluteEncoderType getType() {
        return SwerveAbsoluteEncoderType.CANCODER;
    }

    @Override
    public Object getRaw() {
        return canCoder;
    }
}
