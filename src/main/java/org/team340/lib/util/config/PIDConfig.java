package org.team340.lib.util.config;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Simple config for storing PID constants.
 */
public record PIDConfig(double p, double i, double d, double iZone) {
    public PIDConfig(double p, double i, double d) {
        this(p, i, d, 0.0);
    }

    /**
     * Creates a {@link PIDController} from the config.
     * @return
     */
    public PIDController pidController() {
        PIDController controller = new PIDController(p, i, d);
        controller.setIZone(iZone);
        return controller;
    }

    /**
     * Creates a {@link ProfiledPIDController} from the config.
     * @param constraints Velocity and acceleration constraints for goal.
     */
    public ProfiledPIDController profiledPIDController(TrapezoidProfile.Constraints constraints) {
        ProfiledPIDController controller = new ProfiledPIDController(p, i, d, constraints);
        controller.setIZone(iZone);
        return controller;
    }
}
