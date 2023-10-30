package org.team340.lib.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A modified {@link CommandXboxController}.
 */
public class AdvancedController extends CommandXboxController implements Sendable {

    private final double joystickDeadband;
    private final double joystickThreshold;
    private final double triggerDeadband;
    private final double triggerThreshold;

    /**
     * Create the controller. Uses a deadband of {@code 0.1} and a threshold of {@code 0.5} for the joysticks and triggers.
     * @param port The controller's port.
     */
    public AdvancedController(int port) {
        this(port, 0.1, 0.5, 0.1, 0.5);
    }

    /**
     * Create the controller. Uses a threshold of {@code 0.5} for the joysticks and triggers.
     * @param port The controller's port.
     * @param joystickDeadband A deadband to apply to the joysticks.
     * @param triggerDeadband A deadband to apply to the triggers.
     */
    public AdvancedController(int port, double joystickDeadband, double triggerDeadband) {
        this(port, joystickDeadband, 0.5, triggerDeadband, 0.5);
    }

    /**
     * Create the controller.
     * @param port The controller's port.
     * @param joystickDeadband A deadband to apply to the joysticks.
     * @param joystickThreshold A threshold for joystick event instances.
     * @param triggerDeadband A deadband to apply to the triggers.
     * @param triggerThreshold A threshold for trigger event instances.
     */
    public AdvancedController(
        int port,
        double joystickDeadband,
        double joystickThreshold,
        double triggerDeadband,
        double triggerThreshold
    ) {
        super(port);
        this.joystickDeadband = joystickDeadband;
        this.joystickThreshold = joystickThreshold;
        this.triggerDeadband = triggerDeadband;
        this.triggerThreshold = triggerThreshold;
    }

    /**
     * Constructs an event instance around the up position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickUp() {
        return leftJoystickUp(joystickThreshold);
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
        return leftJoystickDown(joystickThreshold);
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
        return leftJoystickLeft(joystickThreshold);
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
        return leftJoystickRight(joystickThreshold);
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
        return rightJoystickUp(joystickThreshold);
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
        return rightJoystickDown(joystickThreshold);
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
        return rightJoystickLeft(joystickThreshold);
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
        return rightJoystickRight(joystickThreshold);
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
        return super.leftTrigger(triggerThreshold);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger.
     * @return A {@link Trigger} instance.
     */
    @Override
    public Trigger rightTrigger() {
        return super.rightTrigger(triggerThreshold);
    }

    /**`
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of left joystick.
     * @return The axis value.
     */
    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), joystickDeadband);
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of left joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @return The axis value.
     */
    public double getLeftX(double multiplier) {
        return getLeftX() * multiplier;
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of left joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @param exp An exponential factor to apply to the value of the joystick. Applied before the multiplier.
     * @return The axis value.
     */
    public double getLeftX(double multiplier, double exp) {
        if (exp == 1.0) return getLeftX(multiplier);

        double val = getLeftX();
        if (val == 0.0) return val;

        double norm = Math.hypot(val, getLeftY());
        double expMultiplier = exp == 2.0 ? norm : (exp == 3.0 ? norm * norm : Math.pow(norm, exp - 1.0));

        return val * expMultiplier * multiplier;
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of left joystick.
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), joystickDeadband);
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of left joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @return The axis value.
     */
    public double getLeftY(double multiplier) {
        return getLeftY() * multiplier;
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of left joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @param exp An exponential factor to apply to the value of the joystick. Applied before the multiplier.
     * @return The axis value.
     */
    public double getLeftY(double multiplier, double exp) {
        if (exp == 1.0) return getLeftY(multiplier);

        double val = getLeftY();
        if (val == 0.0) return val;

        double norm = Math.hypot(val, getLeftX());
        double expMultiplier = exp == 2.0 ? norm : (exp == 3.0 ? norm * norm : Math.pow(norm, exp - 1.0));

        return val * expMultiplier * multiplier;
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of right joystick.
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), joystickDeadband);
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of right joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @return The axis value.
     */
    public double getRightX(double multiplier) {
        return getRightX() * multiplier;
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of right joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @param exp An exponential factor to apply to the value of the joystick. Applied before the multiplier.
     * @return The axis value.
     */
    public double getRightX(double multiplier, double exp) {
        if (exp == 1.0) return getRightX(multiplier);

        double val = getRightX();
        if (val == 0.0) return val;

        double norm = Math.hypot(val, getRightY());
        double expMultiplier = exp == 2.0 ? norm : (exp == 3.0 ? norm * norm : Math.pow(norm, exp - 1.0));

        return val * expMultiplier * multiplier;
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of right joystick.
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), joystickDeadband);
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of right joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @return The axis value.
     */
    public double getRightY(double multiplier) {
        return getRightY() * multiplier;
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of right joystick.
     * @param multiplier A multiplier for the value of the joystick.
     * @param exp An exponential factor to apply to the value of the joystick. Applied before the multiplier.
     * @return The axis value.
     */
    public double getRightY(double multiplier, double exp) {
        if (exp == 1.0) return getRightY(multiplier);

        double val = getRightY();
        if (val == 0.0) return val;

        double norm = Math.hypot(val, getRightX());
        double expMultiplier = exp == 2.0 ? norm : (exp == 3.0 ? norm * norm : Math.pow(norm, exp - 1.0));

        return val * expMultiplier * multiplier;
    }

    /**
     * Gets the left trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @return The axis value.
     */
    @Override
    public double getLeftTriggerAxis() {
        return Math.max(MathUtil.applyDeadband(super.getLeftTriggerAxis(), triggerDeadband), 0.0);
    }

    /**
     * Gets the left trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @param multiplier A multiplier for the value of the trigger.
     * @return The axis value.
     */
    public double getLeftTriggerAxis(double multiplier) {
        return getLeftTriggerAxis() * multiplier;
    }

    /**
     * Gets the left trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @param multiplier A multiplier for the value of the trigger.
     * @param exp An exponential factor to apply to the value of the trigger. Applied before the multiplier.
     * @return The axis value.
     */
    public double getLeftTriggerAxis(double multiplier, double exp) {
        if (exp == 1.0) return getLeftTriggerAxis(multiplier);
        double raw = getLeftTriggerAxis();
        return (exp == 2.0 ? raw * raw : (exp == 3.0 ? raw * raw * raw : Math.pow(raw, exp))) * multiplier;
    }

    /**
     * Gets the right trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @return The axis value.
     */
    @Override
    public double getRightTriggerAxis() {
        return Math.max(MathUtil.applyDeadband(super.getRightTriggerAxis(), triggerDeadband), 0.0);
    }

    /**
     * Gets the right trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @param multiplier A multiplier for the value of the trigger.
     * @return The axis value.
     */
    public double getRightTriggerAxis(double multiplier) {
        return getRightTriggerAxis() * multiplier;
    }

    /**
     * Gets the right trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @param multiplier A multiplier for the value of the trigger.
     * @param exp An exponential factor to apply to the value of the trigger. Applied before the multiplier.
     * @return The axis value.
     */
    public double getRightTriggerAxis(double multiplier, double exp) {
        if (exp == 1.0) return getRightTriggerAxis(multiplier);
        double raw = getRightTriggerAxis();
        return (exp == 2.0 ? raw * raw : (exp == 3.0 ? raw * raw * raw : Math.pow(raw, exp))) * multiplier;
    }

    /**
     * Gets the difference between the trigger values ({@code left - right}).
     * @return The difference between the trigger values.
     */
    public double getTriggerDifference() {
        return getLeftTriggerAxis() - getRightTriggerAxis();
    }

    /**
     * Gets the difference between the trigger values ({@code left - right}).
     * @param multiplier A multiplier for the difference of the triggers.
     * @return The difference between the trigger values.
     */
    public double getTriggerDifference(double multiplier) {
        return getTriggerDifference() * multiplier;
    }

    /**
     * Gets the difference between the trigger values ({@code left - right}).
     * @param multiplier A multiplier for the difference of the triggers.
     * @param exp An exponential factor to apply to the difference of the triggers. Applied before the multiplier.
     * @return The difference between the trigger values.
     */
    public double getTriggerDifference(double multiplier, double exp) {
        if (exp == 1.0) return getTriggerDifference(multiplier);
        double val = getTriggerDifference();
        if (exp == 2.0) return Math.copySign(val * val, val) * multiplier;
        if (exp == 3.0) return val * val * val * multiplier;
        return Math.copySign(Math.pow(Math.abs(val), exp), val) * multiplier;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        XboxController hid = getHID();

        builder.publishConstString(".api", "AdvancedController");

        builder.addBooleanProperty("a", hid::getAButton, null);
        builder.addBooleanProperty("b", hid::getBButton, null);
        builder.addBooleanProperty("x", hid::getXButton, null);
        builder.addBooleanProperty("y", hid::getYButton, null);

        builder.addBooleanProperty("back", hid::getBackButton, null);
        builder.addBooleanProperty("start", hid::getStartButton, null);

        builder.addIntegerProperty("pov", hid::getPOV, null);

        builder.addBooleanProperty("ls", hid::getLeftStickButton, null);
        builder.addDoubleProperty("lx", () -> Math2.toFixed(hid.getLeftX()), null);
        builder.addDoubleProperty("ly", () -> Math2.toFixed(hid.getLeftY()), null);

        builder.addBooleanProperty("rs", hid::getRightStickButton, null);
        builder.addDoubleProperty("rx", () -> Math2.toFixed(hid.getRightX()), null);
        builder.addDoubleProperty("ry", () -> Math2.toFixed(hid.getRightY()), null);

        builder.addBooleanProperty("lb", hid::getLeftBumper, null);
        builder.addBooleanProperty("rb", hid::getRightBumper, null);

        builder.addDoubleProperty("lt", () -> Math2.toFixed(hid.getLeftTriggerAxis()), null);
        builder.addDoubleProperty("rt", () -> Math2.toFixed(hid.getRightTriggerAxis()), null);
    }
}
