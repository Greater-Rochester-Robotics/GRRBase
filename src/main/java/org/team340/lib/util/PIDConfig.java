package org.team340.lib.util;

/**
 * Simple config for storing PID constants.
 */
public record PIDConfig(double p, double i, double d, double iZone) {
    public PIDConfig(double p, double i, double d) {
        this(p, i, d, 0.0);
    }
}
