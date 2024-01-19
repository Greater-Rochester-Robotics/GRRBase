package org.team340.lib.swerve.config;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.MissingResourceException;
import org.team340.lib.swerve.SwerveBase.SwerveEncoderType;

/**
 * Config builder for {@link SwerveBase} modules.
 */
public class SwerveModuleConfig {

    private String label;
    private SwerveEncoderType encoderType;
    private int encoderDeviceId = -1;
    private String encoderCanBus;
    private double encoderOffset = 0;
    private boolean encoderInverted = false;
    private Translation2d position;
    private int moveMotorDeviceId = -1;
    private String moveMotorCanBus = "";
    private boolean moveMotorBrake;
    private boolean moveMotorInverted;
    private int turnMotorDeviceId = -1;
    private String turnMotorCanBus = "";
    private boolean turnMotorBrake;
    private boolean turnMotorInverted;

    /**
     * Sets the module's label.
     * @param label The label to use. Shown in the dashboard.
     */
    public SwerveModuleConfig setLabel(String label) {
        this.label = label;
        return this;
    }

    /**
     * Gets the configured label.
     */
    public String getLabel() {
        return label;
    }

    /**
     * Use a CANcoder.
     * A simple method for finding {@code offset} is starting with a value of {@code 0.0}, turn the modules manually to face forward, then directly copy the values found in network tables.
     * @param deviceId The device's ID on the CAN bus.
     * @param offset The encoder's offset in radians.
     * @param inverted If the encoder is inverted.
     */
    public SwerveModuleConfig useCANcoder(int deviceId, double offset, boolean flipped) {
        return useCANcoder(deviceId, "", offset, flipped);
    }

    /**
     * Use a CANcoder.
     * A simple method for finding {@code offset} is starting with a value of {@code 0.0}, turn the modules manually to face forward, then directly copy the values found in network tables.
     * @param deviceId The device's ID on the CAN bus.
     * @param canBus The name of the CAN bus being used.
     * @param offset The encoder's offset in radians.
     * @param inverted If the encoder is inverted.
     */
    public SwerveModuleConfig useCANcoder(int deviceId, String canBus, double offset, boolean inverted) {
        encoderType = SwerveEncoderType.CANCODER;
        encoderDeviceId = deviceId;
        encoderCanBus = canBus;
        encoderOffset = offset;
        encoderInverted = inverted;
        return this;
    }

    /**
     * Use a Spark Attached Absolute Encoder.
     * A simple method for finding {@code offset} is starting with a value of {@code 0.0}, turn the modules manually to face forward, then directly copy the values found in network tables.
     * @param offset The encoder's offset in radians.
     * @param inverted If the encoder is inverted.
     */
    public SwerveModuleConfig useSparkAttachedEncoder(double offset, boolean inverted) {
        encoderType = SwerveEncoderType.SPARK_ENCODER;
        encoderOffset = offset;
        encoderInverted = inverted;
        return this;
    }

    /**
     * Gets the selected absolute encoder's type.
     */
    public SwerveEncoderType getEncoderType() {
        return encoderType;
    }

    /**
     * Gets the configured absolute encoder's device ID on the CAN bus if applicable.
     */
    public int getEncoderDeviceId() {
        return encoderDeviceId;
    }

    /**
     * Gets the name of the CAN bus being used by the absolute encoder if applicable.
     */
    public String getEncoderCanBus() {
        return encoderCanBus;
    }

    /**
     * Gets the configured absolute encoder's offset in radians.
     */
    public double getEncoderOffset() {
        return encoderOffset;
    }

    /**
     * Gets the configured inverted state of the absolute encoder.
     */
    public boolean getEncoderInverted() {
        return encoderInverted;
    }

    /**
     * Sets the module's position relative to the center of the robot (in meters).
     * Reminder that relative to the front of the robot, X is forwards and Y is sideways.
     * This can be found from CAD.
     * @param x The module's X position in meters.
     * @param y The module's Y position in meters.
     */
    public SwerveModuleConfig setPosition(double x, double y) {
        position = new Translation2d(x, y);
        return this;
    }

    /**
     * Gets the module's configured position.
     */
    public Translation2d getPosition() {
        return position;
    }

    /**
     * Sets the move motor for the module.
     * @param deviceId The device's ID on the CAN bus.
     * @param brake If brake mode should be enabled. Brake mode is typically enabled.
     * @param inverted If the motor's output is inverted.
     */
    public SwerveModuleConfig setMoveMotor(int deviceId, boolean brake, boolean inverted) {
        moveMotorDeviceId = deviceId;
        moveMotorBrake = brake;
        moveMotorInverted = inverted;
        return this;
    }

    /**
     * Sets the move motor for the module.
     * @param deviceId The device's ID on the CAN bus.
     * @param canBus The name of the CAN bus being used.
     * @param brake If brake mode should be enabled. Brake mode is typically enabled.
     * @param inverted If the motor's output is inverted.
     */
    public SwerveModuleConfig setMoveMotor(int deviceId, String canBus, boolean brake, boolean inverted) {
        moveMotorDeviceId = deviceId;
        moveMotorCanBus = canBus;
        moveMotorBrake = brake;
        moveMotorInverted = inverted;
        return this;
    }

    /**
     * Gets the configured move motor's device ID on the CAN bus.
     */
    public int getMoveMotorDeviceId() {
        return moveMotorDeviceId;
    }

    /**
     * Gets the string of the CAN bus configured to be utilized by the move motor.
     */
    public String getMoveMotorCanBus() {
        return moveMotorCanBus;
    }

    /**
     * Gets the configured brake mode of the move motor.
     */
    public boolean getMoveMotorBrake() {
        return moveMotorBrake;
    }

    /**
     * Gets the configured inverted state of the move motor.
     */
    public boolean getMoveMotorInverted() {
        return moveMotorInverted;
    }

    /**
     * Sets the turn motor for the module.
     * @param deviceId The device's ID on the CAN bus.
     * @param brake If brake mode should be enabled. Brake mode is typically disabled.
     * @param inverted If the motor's output is inverted.
     */
    public SwerveModuleConfig setTurnMotor(int deviceId, boolean brake, boolean inverted) {
        turnMotorDeviceId = deviceId;
        turnMotorBrake = brake;
        turnMotorInverted = inverted;
        return this;
    }

    /**
     * Sets the turn motor for the module.
     * @param deviceId The device's ID on the CAN bus.
     * @param canBus The name of the CAN bus being used.
     * @param brake If brake mode should be enabled. Brake mode is typically disabled.
     * @param inverted If the motor's output is inverted.
     */
    public SwerveModuleConfig setTurnMotor(int deviceId, String canBus, boolean brake, boolean inverted) {
        turnMotorDeviceId = deviceId;
        turnMotorCanBus = canBus;
        turnMotorBrake = brake;
        turnMotorInverted = inverted;
        return this;
    }

    /**
     * Gets the configured turn motor's device ID on the CAN bus.
     */
    public int getTurnMotorDeviceId() {
        return turnMotorDeviceId;
    }

    /**
     * Gets the string of the CAN bus configured to be utilized by the turn motor.
     */
    public String getTurnMotorCanBus() {
        return turnMotorCanBus;
    }

    /**
     * Gets the configured brake mode of the turn motor.
     */
    public boolean getTurnMotorBrake() {
        return turnMotorBrake;
    }

    /**
     * Gets the configured inverted state of the turn motor.
     */
    public boolean getTurnMotorInverted() {
        return turnMotorInverted;
    }

    /**
     * Verifies the config.
     */
    public void verify() {
        if (label == null) throwMissing("Label");
        if (encoderType == null) throwMissing("Encoder Type");
        if (encoderType.equals(SwerveEncoderType.CANCODER)) {
            if (encoderDeviceId == -1) throwMissing("CANcoder Device ID");
            if (encoderCanBus == null) throwMissing("CANcoder CAN Bus");
        }
        if (position == null) throwMissing("Position");
        if (moveMotorDeviceId == -1) throwMissing("Move Motor Device ID");
        if (turnMotorDeviceId == -1) throwMissing("Turn Motor Device ID");
    }

    private void throwMissing(String key) {
        throw new MissingResourceException("Missing value", this.getClass().getSimpleName(), key);
    }
}
