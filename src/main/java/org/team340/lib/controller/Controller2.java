package org.team340.lib.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A modified {@link CommandXboxController}.
 */
public class Controller2 extends CommandXboxController {

    private final Controller2Config config;
    private final JoystickProfile leftProfile;
    private final JoystickProfile rightProfile;

    /**
     * Create the controller.
     * @param config The controller's config.
     */
    public Controller2(Controller2Config config) {
        super(config.getPort());
        this.config = config;
        this.leftProfile = config.getLeftProfile().isEmpty() ? null : JoystickProfile.fromFile(config.getLeftProfile());
        this.rightProfile = config.getRightProfile().isEmpty() ? null : JoystickProfile.fromFile(config.getRightProfile());
    }

    /**
     * Gets the controller's configured label.
     */
    public String getLabel() {
        return config.getLabel();
    }

    /**
     * Constructs an event instance around the up position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickUp() {
        return leftJoystickUp(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the up position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickUp(double threshold) {
        return new Trigger(() -> super.getLeftY() < -threshold);
    }

    /**
     * Constructs an event instance around the down position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickDown() {
        return leftJoystickDown(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the down position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickDown(double threshold) {
        return new Trigger(() -> super.getLeftY() > threshold);
    }

    /**
     * Constructs an event instance around the left position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickLeft() {
        return leftJoystickLeft(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the left position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickLeft(double threshold) {
        return new Trigger(() -> super.getLeftX() < -threshold);
    }

    /**
     * Constructs an event instance around the right position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickRight() {
        return leftJoystickRight(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the right position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickRight(double threshold) {
        return new Trigger(() -> super.getLeftX() > threshold);
    }

    /**
     * Constructs an event instance around the up position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickUp() {
        return rightJoystickUp(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the up position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickUp(double threshold) {
        return new Trigger(() -> super.getRightY() < -threshold);
    }

    /**
     * Constructs an event instance around the down position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickDown() {
        return rightJoystickDown(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the down position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickDown(double threshold) {
        return new Trigger(() -> super.getRightY() > threshold);
    }

    /**
     * Constructs an event instance around the left position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickLeft() {
        return rightJoystickLeft(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the left position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickLeft(double threshold) {
        return new Trigger(() -> super.getRightX() < -threshold);
    }

    /**
     * Constructs an event instance around the right position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickRight() {
        return rightJoystickRight(config.getJoystickThreshold());
    }

    /**
     * Constructs an event instance around the right position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickRight(double threshold) {
        return new Trigger(() -> super.getRightX() > threshold);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger.
     * @return A {@link Trigger} instance.
     */
    @Override
    public Trigger leftTrigger() {
        return super.leftTrigger(config.getTriggerThreshold());
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger.
     * @return A {@link Trigger} instance.
     */
    @Override
    public Trigger rightTrigger() {
        return super.rightTrigger(config.getTriggerThreshold());
    }

    /**`
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of left joystick.
     * @return The axis value.
     */
    @Override
    public double getLeftX() {
        double raw = super.getLeftX();
        return MathUtil.applyDeadband(leftProfile == null ? raw : leftProfile.getX(raw, super.getLeftY()), config.getJoystickDeadband());
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of left joystick.
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        double raw = super.getLeftY();
        return MathUtil.applyDeadband(leftProfile == null ? raw : leftProfile.getY(super.getLeftX(), raw), config.getJoystickDeadband());
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of right joystick.
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        double raw = super.getRightX();
        return MathUtil.applyDeadband(rightProfile == null ? raw : rightProfile.getX(raw, super.getRightY()), config.getJoystickDeadband());
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of right joystick.
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        double raw = super.getRightY();
        return MathUtil.applyDeadband(rightProfile == null ? raw : rightProfile.getY(super.getRightX(), raw), config.getJoystickDeadband());
    }

    /**
     * Gets the left trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @return The axis value.
     */
    @Override
    public double getLeftTriggerAxis() {
        return Math.max(MathUtil.applyDeadband(super.getLeftTriggerAxis(), config.getTriggerDeadband()), 0.0);
    }

    /**
     * Gets the right trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @return The axis value.
     */
    @Override
    public double getRightTriggerAxis() {
        return Math.max(MathUtil.applyDeadband(super.getRightTriggerAxis(), config.getTriggerDeadband()), 0.0);
    }

    /**
     * Gets the difference between the trigger values ({@code left - right}).
     * @return The difference between the trigger values.
     */
    public double getTriggerDifference() {
        return getLeftTriggerAxis() - getRightTriggerAxis();
    }
}
