package org.team340.lib.swerve.hardware.encoders.vendors;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import org.team340.lib.swerve.SwerveBase.SwerveAbsoluteEncoderType;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;

/**
 * Wrapper for an absolute encoder attached directly to a REV Spark Max for swerve (Through bor, MagEncoder with adapter board, CANandcoder, etc).
 */
public class SwerveSparkMaxEncoder extends SwerveEncoder {

    private final SparkMaxAbsoluteEncoder sparkMaxEncoder;

    /**
     * Create the Spark Max Attached Encoder wrapper.
     * @param sparkMaxEncoder The encoder to wrap.
     * @param config The general swerve config.
     * @param moduleConfig The encoder's module's config.
     */
    public SwerveSparkMaxEncoder(SparkMaxAbsoluteEncoder sparkMaxEncoder) {
        super(SwerveAbsoluteEncoderType.SPARK_MAX_ENCODER);
        this.sparkMaxEncoder = sparkMaxEncoder;
        // Config options are applied in SwerveSparkMax.
    }

    @Override
    protected double getRealPosition() {
        return MathUtil.angleModulus(sparkMaxEncoder.getPosition());
    }
}
