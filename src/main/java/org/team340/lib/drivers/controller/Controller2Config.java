package org.team340.lib.drivers.controller;

/**
 * Config builder for {@link Controller2}.
 */
public class Controller2Config {

    private String label = "Controller";
    private int port = -1;
    private double joystickDeadband = 0.0;
    private double joystickThreshold = 0.0;
    private double triggerDeadband = 0.0;
    private double triggerThreshold = 0.0;
    private String leftProfile = "";
    private String rightProfile = "";

    /**
     * Sets the controller's label.
     * Used in network tables.
     * @param label The label to use.
     */
    public Controller2Config setLabel(String label) {
        this.label = label;
        return this;
    }

    /**
     * Gets the controller's configured label.
     */
    public String getLabel() {
        return label;
    }

    /**
     * Sets the port the controller is connected to.
     * @param port The port the controller is connected to.
     */
    public Controller2Config setPort(int port) {
        this.port = port;
        return this;
    }

    /**
     * Gets the controller's configured port.
     */
    public int getPort() {
        return port;
    }

    /**
     * Sets the deadband of the controller's joysticks.
     * @param joystickDeadband The deadband to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public Controller2Config setJoystickDeadband(double joystickDeadband) {
        this.joystickDeadband = joystickDeadband;
        return this;
    }

    /**
     * Gets the configured joystick deadband.
     */
    public double getJoystickDeadband() {
        return joystickDeadband;
    }

    /**
     * Sets the threshold of the controller's joysticks when being used as "buttons".
     * @param joystickThreshold The threshold to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public Controller2Config setJoystickThreshold(double joystickThreshold) {
        this.joystickThreshold = joystickThreshold;
        return this;
    }

    /**
     * Gets the configured joystick threshold.
     */
    public double getJoystickThreshold() {
        return joystickThreshold;
    }

    /**
     * Sets the deadband of the controller's triggers.
     * @param triggerDeadband The deadband to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public Controller2Config setTriggerDeadband(double triggerDeadband) {
        this.triggerDeadband = triggerDeadband;
        return this;
    }

    /**
     * Gets the configured trigger deadband.
     */
    public double getTriggerDeadband() {
        return triggerDeadband;
    }

    /**
     * Sets the threshold of the controller's triggers when being used as "buttons".
     * @param triggerThreshold The threshold to use. Should be a value from {@code 0.0} to {@code 1.0}.
     */
    public Controller2Config setTriggerThreshold(double triggerThreshold) {
        this.triggerThreshold = triggerThreshold;
        return this;
    }

    /**
     * Gets the configured trigger threshold.
     */
    public double getTriggerThreshold() {
        return triggerThreshold;
    }

    /**
     * Sets the profile file for the controller's left joystick.
     * An empty string disables the joystick profile.
     * @param leftProfile The file path for the controller's left joystick profile.
     */
    public Controller2Config setLeftProfile(String leftProfile) {
        this.leftProfile = leftProfile;
        return this;
    }

    /**
     * Gets the file path of the controller's left joystick profile.
     */
    public String getLeftProfile() {
        return leftProfile;
    }

    /**
     * Sets the profile file for the controller's right joystick.
     * An empty string disables the joystick profile.
     * @param rightProfile The file path for the controller's left joystick profile.
     */
    public Controller2Config setRightProfile(String rightProfile) {
        this.rightProfile = rightProfile;
        return this;
    }

    /**
     * Gets the file path of the controller's right joystick profile.
     */
    public String getRightProfile() {
        return rightProfile;
    }
}
