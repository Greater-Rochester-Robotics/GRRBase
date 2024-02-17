package org.team340.lib.util.config.rev;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import org.team340.lib.util.Math2;

/**
 * Config builder for {@link RelativeEncoder}.
 */
public final class RelativeEncoderConfig extends RevConfigBase<RelativeEncoder> {

    /**
     * Creates an empty config.
     */
    public RelativeEncoderConfig() {}

    /**
     * Creates a config that copies the config steps from the base provided.
     * @param base The config to copy the steps from.
     */
    private RelativeEncoderConfig(RevConfigBase<RelativeEncoder> base) {
        super(base);
    }

    /**
     * Clones this config.
     */
    public RelativeEncoderConfig clone() {
        return new RelativeEncoderConfig(this);
    }

    /**
     * Applies the config to a Spark Max attached encoder.
     * @param sparkMax The Spark Max the encoder is attached to.
     * @param relativeEncoder The relative encoder.
     */
    public void apply(CANSparkMax sparkMax, RelativeEncoder relativeEncoder) {
        super.applySteps(relativeEncoder, "Spark Max (ID " + sparkMax.getDeviceId() + ") Relative Encoder");
    }

    /**
     * Applies the config to a Spark Flex attached encoder.
     * @param sparkFlex The Spark Flex the encoder is attached to.
     * @param relativeEncoder The relative encoder.
     */
    public void apply(CANSparkFlex sparkFlex, RelativeEncoder relativeEncoder) {
        super.applySteps(relativeEncoder, "Spark Flex (ID " + sparkFlex.getDeviceId() + ") Relative Encoder");
    }

    /**
     * Sets the sampling depth of the velocity calculation process for a quadrature or hall sensor
     * encoder. This value sets the number of samples in the average for velocity readings. For a
     * quadrature encoder, this can be any value from {@code 1} to {@code 64} (default). For a hall
     * sensor, it must be either {@code 1}, {@code 2}, {@code 4}, or {@code 8} (default).
     * @param depth The velocity calculation process's sampling depth.
     */
    public RelativeEncoderConfig setAverageDepth(int depth) {
        addStep(
            relativeEncoder -> relativeEncoder.setAverageDepth(depth),
            relativeEncoder -> relativeEncoder.getAverageDepth() == depth,
            "Average Depth"
        );
        return this;
    }

    /**
     * Sets the phase of the motor feedback sensor so that it is set to be in phase with the motor itself.
     * This only works for quadrature encoders and analog sensors. This will throw an error if the user
     * tries to set the inversion of the hall sensor.
     * @param inverted The phase of the sensor.
     */
    public RelativeEncoderConfig setInverted(boolean inverted) {
        addStep(
            relativeEncoder -> relativeEncoder.setInverted(inverted),
            relativeEncoder -> relativeEncoder.getInverted() == inverted,
            "Inverted"
        );
        return this;
    }

    /**
     * Sets the position measurement period used to calculate the velocity of a quadrature or hall sensor
     * encoder. For a quadrature encoder, this number may be between {@code 1} and {@code 100} (default). For
     * a hall sensor, this number may be between {@code 8} and {@code 64}. The default for a hall sensor is 32ms.
     * @param periodMs Measurement period in milliseconds.
     */
    public RelativeEncoderConfig setMeasurementPeriod(int periodMs) {
        addStep(
            relativeEncoder -> relativeEncoder.setMeasurementPeriod(periodMs),
            relativeEncoder -> relativeEncoder.getMeasurementPeriod() == periodMs,
            "Measurement Period"
        );
        return this;
    }

    /**
     * Sets the conversion factor for position of the encoder. Multiplied
     * by the native output units to give you position.
     * @param factor The conversion factor to multiply the native units by.
     */
    public RelativeEncoderConfig setPositionConversionFactor(double factor) {
        addStep(
            relativeEncoder -> relativeEncoder.setPositionConversionFactor(factor),
            relativeEncoder -> Math2.epsilonEquals(relativeEncoder.getPositionConversionFactor(), factor, RevConfigRegistry.EPSILON),
            "Position Conversion Factor"
        );
        return this;
    }

    /**
     * Sets the conversion factor for velocity of the encoder. Multiplied
     * by the native output units to give you velocity.
     * @param factor The conversion factor to multiply the native units by.
     */
    public RelativeEncoderConfig setVelocityConversionFactor(double factor) {
        addStep(
            relativeEncoder -> relativeEncoder.setVelocityConversionFactor(factor),
            relativeEncoder -> Math2.epsilonEquals(relativeEncoder.getVelocityConversionFactor(), factor, RevConfigRegistry.EPSILON),
            "Velocity Conversion Factor"
        );
        return this;
    }
}
