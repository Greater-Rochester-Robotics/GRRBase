package org.team340.lib.controller;

/**
 * Config builder for {@link Controller}.
 */
public class ControllerConfig {

    /** The port number the controller is connected to in the DS. */
    public int port = 0;
    /** The configured deadband for the controller's joysticks. */
    public double joystickDeadband = 0.1;
    /** The configured deadband for the controller's triggers. */
    public double triggerDeadband = 0.1;
    /** The configured threshold for "button" usage of the controller's joysticks. */
    public double joystickThreshold = 0.5;
    /** The configured threshold for "button" usage of the controller's triggers. */
    public double triggerThreshold = 0.5;

    /**
     * Sets the port the controller is connected to in the DS.
     * @param port The port the controller is connected to.
     */
    public ControllerConfig setPort(int port) {
        this.port = port;
        return this;
    }

    /**
     * Sets the deadband of the controller's joysticks and triggers.
     * @param joysticks The deadband to use for joysticks. Should be a value from {@code 0.0} to {@code 1.0}.
     * @param triggers The deadband to use for triggers. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public ControllerConfig setDeadbands(double joysticks, double triggers) {
        joystickDeadband = joysticks;
        triggerDeadband = triggers;
        return this;
    }

    /**
     * Sets the threshold of the controller's joysticks and triggers when being used as "buttons".
     * @param joysticks The threshold to use for joysticks. Should be a value from {@code 0.0} to {@code 1.0}.
     * @param triggers The threshold to use for triggers. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public ControllerConfig setThresholds(double joysticks, double triggers) {
        joystickThreshold = joysticks;
        triggerThreshold = triggers;
        return this;
    }
}
