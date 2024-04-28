package org.team340.lib.util.config;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * Simple config for storing common gains for feed forward.
 */
public record FeedForwardConfig(double s, double v, double a, double g) {
    public FeedForwardConfig(double v) {
        this(0.0, v, 0.0, 0.0);
    }

    public FeedForwardConfig(double s, double v) {
        this(s, v, 0.0);
    }

    public FeedForwardConfig(double s, double v, double a) {
        this(s, v, a, 0.0);
    }

    /**
     * Creates an {@link ArmFeedForward} from the config.
     */
    public ArmFeedforward armFeedForward() {
        return new ArmFeedforward(s, g, v, a);
    }

    /**
     * Creates an {@link ElevatorFeedForward} from the config.
     */
    public ElevatorFeedforward elevatorFeedForward() {
        return new ElevatorFeedforward(s, g, v, a);
    }

    /**
     * Creates a {@link SimpleMotorFeedForward} from the config.
     */
    public SimpleMotorFeedforward simpleMotorFeedForward() {
        return new SimpleMotorFeedforward(s, v, a);
    }
}
