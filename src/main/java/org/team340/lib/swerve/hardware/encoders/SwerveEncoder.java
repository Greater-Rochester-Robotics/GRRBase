package org.team340.lib.swerve.hardware.encoders;

/**
 * An absolute encoder wrapper for swerve.
 * Bound to {@link SwerveModule}s.
 */
public interface SwerveEncoder {
    /**
     * Gets the encoder's position in radians.
     * Clamped from {@code -PI} to {@code PI}.
     */
    public abstract double getPosition();
}
