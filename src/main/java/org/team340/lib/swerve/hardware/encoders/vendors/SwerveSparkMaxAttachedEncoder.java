package org.team340.lib.swerve.hardware.encoders.vendors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import org.team340.lib.math.Math2;
import org.team340.lib.swerve.SwerveBase.SwerveAbsoluteEncoderType;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveAbsoluteEncoder;
import org.team340.lib.util.RevUtil;

/**
 * Wrapper for an absolute encoder attached directly to a REV Spark Max for swerve (Through bor, MagEncoder with adapter board, CANandcoder, etc).
 */
public class SwerveSparkMaxAttachedEncoder implements SwerveAbsoluteEncoder {

    /**
     * The module's config.
     */
    private final SwerveModuleConfig moduleConfig;
    /**
     * The absolute encoder.
     */
    private final SparkMaxAbsoluteEncoder absoluteEncoder;

    /**
     * Create the Spark Max Attached Encoder wrapper.
     * @param absoluteEncoder The absolute encoder to wrap.
     * @param config The general swerve config.
     * @param moduleConfig The encoder's module's config.
     */
    public SwerveSparkMaxAttachedEncoder(SparkMaxAbsoluteEncoder absoluteEncoder, SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.absoluteEncoder = absoluteEncoder;
        this.moduleConfig = moduleConfig;
    }

    /**
     * Configures the encoder.
     * Used by {@link SwerveSparkMax} (Encoder configuration is on the Spark Max).
     * @param sparkMax The Spark Max the absolute encoder is attached to.
     */
    public void config(CANSparkMax sparkMax) {
        RevUtil.AbsoluteEncoder.setPositionConversionFactor(sparkMax, absoluteEncoder, Math2.TWO_PI);
        RevUtil.AbsoluteEncoder.setVelocityConversionFactor(sparkMax, absoluteEncoder, Math2.TWO_PI / 60.0);
        RevUtil.AbsoluteEncoder.setInverted(sparkMax, absoluteEncoder, moduleConfig.getAbsoluteEncoderInverted());
        RevUtil.AbsoluteEncoder.setZeroOffset(sparkMax, absoluteEncoder, moduleConfig.getAbsoluteEncoderOffset());
    }

    @Override
    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    @Override
    public SwerveAbsoluteEncoderType getType() {
        return SwerveAbsoluteEncoderType.SPARK_MAX_ATTACHED;
    }

    @Override
    public Object getRaw() {
        return absoluteEncoder;
    }
}
