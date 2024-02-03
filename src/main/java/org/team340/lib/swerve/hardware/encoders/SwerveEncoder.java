package org.team340.lib.swerve.hardware.encoders;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

/**
 * An absolute encoder wrapper for swerve.
 * Bound to {@link SwerveModule}s.
 */
public interface SwerveEncoder {
    /**
     * Supported encoders.
     */
    public static enum Type {
        CANCODER,
        SPARK_ENCODER,
    }

    /**
     * Gets the encoder's position in radians.
     * Clamped from {@code -PI} to {@code PI}.
     */
    public abstract double getPosition();
}
