package org.team340.lib.controller;

/**
 * Config builder for {@link Controller}.
 */
public class ControllerConfig {

    int port = -1;
    double joystickDeadband = 0.1;
    double joystickThreshold = 0.5;
    double triggerDeadband = 0.1;
    double triggerThreshold = 0.5;

    /**
     * Sets the port the controller is connected to.
     * @param port The port the controller is connected to.
     */
    public ControllerConfig setPort(int port) {
        this.port = port;
        return this;
    }

    /**
     * Sets the deadband of the controller's joysticks.
     * @param value The deadband to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public ControllerConfig setJoystickDeadband(double value) {
        joystickDeadband = value;
        return this;
    }

    /**
     * Sets the threshold of the controller's joysticks when being used as "buttons".
     * @param value The threshold to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public ControllerConfig setJoystickThreshold(double value) {
        joystickThreshold = value;
        return this;
    }

    /**
     * Sets the deadband of the controller's triggers.
     * @param value The deadband to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public ControllerConfig setTriggerDeadband(double value) {
        triggerDeadband = value;
        return this;
    }

    /**
     * Sets the threshold of the controller's triggers when being used as "buttons".
     * @param value The threshold to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public ControllerConfig setTriggerThreshold(double value) {
        triggerThreshold = value;
        return this;
    }
}
