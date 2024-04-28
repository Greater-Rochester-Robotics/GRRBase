package org.team340.lib.swerve.hardware.encoders.vendors;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;

/**
 * Wrapper for an absolute encoder attached directly to a REV Spark for swerve (Through bor, MagEncoder with adapter board, CANandcoder, etc).
 */
public class SwerveSparkEncoder implements SwerveEncoder {

    private final CANSparkBase spark;
    private final SparkAbsoluteEncoder sparkEncoder;

    /**
     * Create the Spark Attached Encoder wrapper.
     * @param spark The Spark the encoder is attached to.
     * @param sparkEncoder The encoder to wrap.
     */
    public SwerveSparkEncoder(CANSparkBase spark, SparkAbsoluteEncoder sparkEncoder) {
        this.spark = spark;
        this.sparkEncoder = sparkEncoder;
        // Config options are applied in SwerveSparkMax / SwerveSparkFlex.
    }

    @Override
    public double getPosition() {
        return MathUtil.angleModulus(sparkEncoder.getPosition());
    }

    @Override
    public boolean readError() {
        return !spark.getLastError().equals(REVLibError.kOk);
    }
}
