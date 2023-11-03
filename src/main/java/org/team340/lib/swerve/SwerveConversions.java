package org.team340.lib.swerve;

import edu.wpi.first.math.util.Units;
import org.team340.lib.math.Math2;
import org.team340.lib.swerve.config.SwerveConfig;

/**
 * Conversions for config defined measurements.
 */
public record SwerveConversions(double moveRotationsPerMeter, double turnRotationsPerRadian) {
    /**
     * Generate conversions from a config.
     * @param config The config to generate from.
     */
    public SwerveConversions(SwerveConfig config) {
        this(
            config.getMoveGearRatio() / (Units.inchesToMeters(config.getWheelDiameterInches()) * Math.PI),
            config.getTurnGearRatio() / Math2.TWO_PI
        );
    }
}
