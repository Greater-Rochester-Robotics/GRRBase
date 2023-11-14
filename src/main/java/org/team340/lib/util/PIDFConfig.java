package org.team340.lib.util;

/**
 * Simple config for storing PID constants.
 */
public record PIDFConfig(double p, double i, double d, double f, double iZone) {
    public PIDFConfig(double p, double i, double d, double f) {
        this(p, i, d, f, 0.0);
    }
}
