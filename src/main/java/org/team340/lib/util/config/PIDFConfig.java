package org.team340.lib.util.config;

/**
 * Simple config for storing PID constants with a feed forward value.
 */
public record PIDFConfig(double p, double i, double d, double ff, double iZone) {
    public PIDFConfig(double p, double i, double d, double ff) {
        this(p, i, d, ff, 0.0);
    }
}
