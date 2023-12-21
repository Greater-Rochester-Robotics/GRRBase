package org.team340.lib.util.config;

/**
 * Simple config for storing common gains for feed forward.
 */
public record FeedForwardConfig(double s, double v, double a, double g) {
    public FeedForwardConfig(double s, double v, double a) {
        this(s, v, a, 0.0);
    }
}
