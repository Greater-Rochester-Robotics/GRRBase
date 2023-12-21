package org.team340.lib.util.config.rev;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import org.team340.lib.util.Math2;

/**
 * Config builder for {@link AbsoluteEncoder}.
 * Use {@link AbsoluteEncoderConfig#apply(CANSparkMax sparkMax, AbsoluteEncoder absoluteEncoder) apply()} to apply the config.
 */
public class AbsoluteEncoderConfig extends RevConfigBase<AbsoluteEncoder> {

    /**
     * Applies the config to a Spark Max attached encoder..
     * @param sparkMax The Spark Max the encoder is attached to.
     * @param absoluteEncoder The absolute encoder.
     */
    public void apply(CANSparkMax sparkMax, AbsoluteEncoder absoluteEncoder) {
        addStep(
            ae -> {
                RevConfigUtils.burnFlashSleep();
                return sparkMax.burnFlash();
            },
            "Burn Flash"
        );
        super.applySteps(absoluteEncoder, "Spark Max (ID " + sparkMax.getDeviceId() + ") Absolute Encoder");
    }

    /**
     * Sets the average sampling depth for an absolute encoder. This is a bit size and should be
     * either {@code 1}, {@code 2}, {@code 4}, {@code 8}, {@code 16}, {@code 32}, {@code 64}, or {@code 128}.
     * @param depth The average sampling depth of {@code 1}, {@code 2}, {@code 4}, {@code 8}, {@code 16}, {@code 32}, {@code 64}, or {@code 128}.
     */
    public AbsoluteEncoderConfig setAverageDepth(int depth) {
        addStep(
            absoluteEncoder -> absoluteEncoder.setAverageDepth(depth),
            absoluteEncoder -> absoluteEncoder.getAverageDepth() == depth,
            "Average Depth"
        );
        return this;
    }

    /**
     * Sets the phase of the absolute encoder so that it is set to be in phase with the motor itself.
     * @param inverted The phase of the encoder.
     */
    public AbsoluteEncoderConfig setInverted(boolean inverted) {
        addStep(
            absoluteEncoder -> absoluteEncoder.setInverted(inverted),
            absoluteEncoder -> absoluteEncoder.getInverted() == inverted,
            "Inverted"
        );
        return this;
    }

    /**
     * Sets the conversion factor for position of the encoder. Multiplied
     * by the native output units to give you position.
     * @param factor The conversion factor to multiply the native units (rotations) by.
     */
    public AbsoluteEncoderConfig setPositionConversionFactor(double factor) {
        addStep(
            absoluteEncoder -> absoluteEncoder.setPositionConversionFactor(factor),
            absoluteEncoder -> Math2.epsilonEquals(absoluteEncoder.getPositionConversionFactor(), factor, RevConfigUtils.EPSILON),
            "Position Conversion Factor"
        );
        return this;
    }

    /**
     * Sets the conversion factor for velocity of the encoder. Multiplied
     * by the native output units to give you velocity.
     * @param factor The conversion factor to multiply the native units (rotations per minute) by.
     */
    public AbsoluteEncoderConfig setVelocityConversionFactor(double factor) {
        addStep(
            absoluteEncoder -> absoluteEncoder.setVelocityConversionFactor(factor),
            absoluteEncoder -> Math2.epsilonEquals(absoluteEncoder.getVelocityConversionFactor(), factor, RevConfigUtils.EPSILON),
            "Velocity Conversion Factor"
        );
        return this;
    }

    /**
     * Sets the zero offset of an absolute encoder (the position that is reported as zero).
     * The zero offset is specified as the reported position of the encoder in the desired
     * zero position, if the zero offset was set to 0. It is influenced by the absolute
     * encoder's position conversion factor, and whether it is inverted. Always call
     * {@code setConversionFactor()} and {@code setInverted()}
     * before calling this function.
     * @param offset The zero offset with the position conversion factor applied.
     */
    public AbsoluteEncoderConfig setZeroOffset(double offset) {
        addStep(
            absoluteEncoder -> absoluteEncoder.setZeroOffset(offset),
            absoluteEncoder -> Math2.epsilonEquals(absoluteEncoder.getZeroOffset(), offset, RevConfigUtils.EPSILON),
            "Zero Offset"
        );
        return this;
    }
}
