package org.team340.lib.swerve.hardware.encoders;

import org.team340.lib.swerve.SwerveBase.SwerveAbsoluteEncoderType;

/**
 * An absolute encoder wrapper for swerve.
 * Bound to {@link SwerveModule}s.
 */
public interface SwerveAbsoluteEncoder {
    /**
     * Gets the encoder's position in radians.
     * Clamped from {@code 0} to {@code 2 PI}.
     */
    public abstract double getAbsolutePosition();

    /**
     * Gets the encoder's type.
     */
    public abstract SwerveAbsoluteEncoderType getType();

    /**
     * Gets the encoder's raw object.
     */
    public abstract Object getRaw();
}
