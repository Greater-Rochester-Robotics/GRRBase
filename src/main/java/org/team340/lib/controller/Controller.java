package org.team340.lib.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team340.lib.util.Profiler;

/**
 * A modified {@link CommandXboxController}.
 * Uses configured deadbands and thresholds when utilizing
 * axis values, and all buttons are disabled when the robot
 * <a href="https://youtu.be/6K7QkXYmEp4?t=14"> is in autonomous mode.</a>
 */
public class Controller extends CommandXboxController {

    private final ControllerConfig config;
    private final EventLoop loop = new EventLoop();

    /**
     * Create the controller.
     * @param config The controller's config.
     */
    public Controller(ControllerConfig config) {
        super(config.port);
        this.config = config;

        CommandScheduler
            .getInstance()
            .getDefaultButtonLoop()
            .bind(() -> {
                boolean profile = Profiler.isRunning();
                if (profile) Profiler.start("Controller" + config.port);
                if (!RobotState.isAutonomous()) loop.poll();
                if (profile) Profiler.end();
            });
    }

    @Override
    public Trigger button(int button) {
        return button(button, loop);
    }

    @Override
    public Trigger pov(int angle) {
        return pov(0, angle, loop);
    }

    @Override
    public Trigger povUp() {
        return pov(0);
    }

    @Override
    public Trigger povUpRight() {
        return pov(45);
    }

    @Override
    public Trigger povRight() {
        return pov(90);
    }

    @Override
    public Trigger povDownRight() {
        return pov(135);
    }

    @Override
    public Trigger povDown() {
        return pov(180);
    }

    @Override
    public Trigger povDownLeft() {
        return pov(225);
    }

    @Override
    public Trigger povLeft() {
        return pov(270);
    }

    @Override
    public Trigger povUpLeft() {
        return pov(315);
    }

    @Override
    public Trigger povCenter() {
        return pov(-1);
    }

    @Override
    public Trigger axisLessThan(int axis, double threshold) {
        return axisLessThan(axis, threshold, loop);
    }

    @Override
    public Trigger axisGreaterThan(int axis, double threshold) {
        return axisGreaterThan(axis, threshold, loop);
    }

    @Override
    public Trigger a() {
        return a(loop);
    }

    @Override
    public Trigger b() {
        return b(loop);
    }

    @Override
    public Trigger x() {
        return x(loop);
    }

    @Override
    public Trigger y() {
        return y(loop);
    }

    @Override
    public Trigger leftBumper() {
        return leftBumper(loop);
    }

    @Override
    public Trigger rightBumper() {
        return rightBumper(loop);
    }

    @Override
    public Trigger back() {
        return back(loop);
    }

    @Override
    public Trigger start() {
        return start(loop);
    }

    @Override
    public Trigger leftStick() {
        return leftStick(loop);
    }

    @Override
    public Trigger rightStick() {
        return rightStick(loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger.
     * @return A {@link Trigger} instance.
     */
    @Override
    public Trigger leftTrigger() {
        return super.leftTrigger(config.triggerThreshold);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger.
     * @return A {@link Trigger} instance.
     */
    @Override
    public Trigger rightTrigger() {
        return super.rightTrigger(config.triggerThreshold);
    }

    /**
     * Constructs an event instance around the up position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickUp() {
        return leftJoystickUp(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the up position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickUp(double threshold) {
        return axisLessThan(Axis.kLeftY.value, -threshold);
    }

    /**
     * Constructs an event instance around the down position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickDown() {
        return leftJoystickDown(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the down position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickDown(double threshold) {
        return axisGreaterThan(Axis.kLeftY.value, threshold);
    }

    /**
     * Constructs an event instance around the left position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickLeft() {
        return leftJoystickLeft(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the left position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickLeft(double threshold) {
        return axisLessThan(Axis.kLeftX.value, -threshold);
    }

    /**
     * Constructs an event instance around the right position of the left joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickRight() {
        return leftJoystickRight(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the right position of the left joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger leftJoystickRight(double threshold) {
        return axisGreaterThan(Axis.kLeftX.value, threshold);
    }

    /**
     * Constructs an event instance around the up position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickUp() {
        return rightJoystickUp(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the up position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickUp(double threshold) {
        return axisLessThan(Axis.kRightY.value, -threshold);
    }

    /**
     * Constructs an event instance around the down position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickDown() {
        return rightJoystickDown(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the down position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickDown(double threshold) {
        return axisGreaterThan(Axis.kRightY.value, threshold);
    }

    /**
     * Constructs an event instance around the left position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickLeft() {
        return rightJoystickLeft(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the left position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickLeft(double threshold) {
        return axisLessThan(Axis.kRightX.value, -threshold);
    }

    /**
     * Constructs an event instance around the right position of the right joystick.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickRight() {
        return rightJoystickRight(config.joystickThreshold);
    }

    /**
     * Constructs an event instance around the right position of the right joystick.
     * @param threshold The minimum axis value for the returned {@link Trigger} to be true. Should be between {@code 0.0} and {@code 1.0}. Deadband is not applied.
     * @return A {@link Trigger} instance.
     */
    public Trigger rightJoystickRight(double threshold) {
        return axisGreaterThan(Axis.kRightX.value, threshold);
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of left joystick.
     * @return The axis value.
     */
    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), config.joystickDeadband);
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of left joystick.
     * @return The axis value.
     */
    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), config.joystickDeadband);
    }

    /**
     * Gets the X axis value (left = {@code -1.0}, right = {@code 1.0}) of right joystick.
     * @return The axis value.
     */
    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), config.joystickDeadband);
    }

    /**
     * Gets the Y axis value (up = {@code -1.0}, down = {@code 1.0}) of right joystick.
     * @return The axis value.
     */
    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), config.joystickDeadband);
    }

    /**
     * Gets the left trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @return The axis value.
     */
    @Override
    public double getLeftTriggerAxis() {
        return MathUtil.applyDeadband(super.getLeftTriggerAxis(), config.triggerDeadband);
    }

    /**
     * Gets the right trigger value (no press = {@code 0.0}, full press = {@code 1.0}).
     * @return The axis value.
     */
    @Override
    public double getRightTriggerAxis() {
        return MathUtil.applyDeadband(super.getRightTriggerAxis(), config.triggerDeadband);
    }

    /**
     * Gets the difference between the trigger values ({@code left - right}).
     * @return The difference between the trigger values.
     */
    public double getTriggerDifference() {
        return getLeftTriggerAxis() - getRightTriggerAxis();
    }
}
