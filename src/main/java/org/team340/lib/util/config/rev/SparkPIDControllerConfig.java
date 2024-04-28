package org.team340.lib.util.config.rev;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.PIDConfig;

/**
 * Config builder for {@link SparkPIDController}.
 */
public final class SparkPIDControllerConfig extends RevConfigBase<SparkPIDController> {

    /**
     * Creates an empty config.
     */
    public SparkPIDControllerConfig() {}

    /**
     * Creates a config that copies the config steps from the base provided.
     * @param base The config to copy the steps from.
     */
    private SparkPIDControllerConfig(RevConfigBase<SparkPIDController> base) {
        super(base);
    }

    /**
     * Clones this config.
     */
    public SparkPIDControllerConfig clone() {
        return new SparkPIDControllerConfig(this);
    }

    /**
     * Applies the config.
     * @param sparkMax The Spark Max to apply the config to.
     * @param pidController The PID controller.
     */
    public void apply(CANSparkMax sparkMax, SparkPIDController pidController) {
        super.applySteps(pidController, "Spark Max (ID " + sparkMax.getDeviceId() + ") PID Controller");
    }

    /**
     * Applies the config.
     * @param sparkFlex The Spark Flex to apply the config to.
     * @param pidController The PID controller.
     */
    public void apply(CANSparkFlex sparkFlex, SparkPIDController pidController) {
        super.applySteps(pidController, "Spark Flex (ID " + sparkFlex.getDeviceId() + ") PID Controller");
    }

    /**
     * Sets the derivative gain constant of the PIDF controller on the Spark Max.
     * @param gain The derivative gain value, must be positive.
     */
    public SparkPIDControllerConfig setD(double gain) {
        addStep(
            pidController -> pidController.setD(gain),
            pidController -> Math2.epsilonEquals(pidController.getD(), gain, RevConfigRegistry.EPSILON),
            "D Gain"
        );
        return this;
    }

    /**
     * Sets the derivative gain constant of the PIDF controller on the Spark Max.
     * @param gain The derivative gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setD(double gain, int slotId) {
        addStep(
            pidController -> pidController.setD(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getD(slotId), gain, RevConfigRegistry.EPSILON),
            "D Gain (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Sets the the derivative filter constant of the PIDF controller on the Spark Max.
     * @param gain The derivative filter value, must be a positive number between {@code 0.0} and {@code 1.0}.
     */
    public SparkPIDControllerConfig setDFilter(double gain) {
        addStep(
            pidController -> pidController.setDFilter(gain),
            pidController -> Math2.epsilonEquals(pidController.getDFilter(0), gain, RevConfigRegistry.EPSILON),
            "D Filter"
        );
        return this;
    }

    /**
     * Sets the the derivative filter constant of the PIDF controller on the Spark Max.
     * @param gain The derivative filter value, must be a positive number between {@code 0.0} and {@code 1.0}.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setDFilter(double gain, int slotId) {
        addStep(
            pidController -> pidController.setDFilter(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getDFilter(slotId), gain, RevConfigRegistry.EPSILON),
            "D Filter (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Sets the controller's feedback device. The default feedback device in brushless mode
     * is assumed to be the integrated encoder and the default feedback device in brushed
     * mode is assumed to be a quadrature encoder. This is used to changed to another
     * feedback device for the controller, such as an analog sensor. If there is a limited
     * range on the feedback sensor that should be observed by the PIDController, it can be
     * set by calling {@code setFeedbackSensorRange()} on the sensor object.
     * @param sensor The sensor to use as a feedback device.
     */
    public SparkPIDControllerConfig setFeedbackDevice(MotorFeedbackSensor sensor) {
        addStep(pidController -> pidController.setFeedbackDevice(sensor), "Feedback Device");
        return this;
    }

    /**
     * Sets the feed-forward gain constant of the PIDF controller on the Spark Max.
     * @param gain The  feed-forward gain value, must be positive.
     */
    public SparkPIDControllerConfig setFF(double gain) {
        addStep(
            pidController -> pidController.setFF(gain),
            pidController -> Math2.epsilonEquals(pidController.getFF(), gain, RevConfigRegistry.EPSILON),
            "FF Gain"
        );
        return this;
    }

    /**
     * Sets the  feed-forward gain constant of the PIDF controller on the Spark Max.
     * @param gain The  feed-forward gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setFF(double gain, int slotId) {
        addStep(
            pidController -> pidController.setFF(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getFF(slotId), gain, RevConfigRegistry.EPSILON),
            "FF Gain (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Sets the integral gain constant of the PIDF controller on the Spark Max.
     * @param gain The integral gain value, must be positive.
     */
    public SparkPIDControllerConfig setI(double gain) {
        addStep(
            pidController -> pidController.setI(gain),
            pidController -> Math2.epsilonEquals(pidController.getI(), gain, RevConfigRegistry.EPSILON),
            "I Gain"
        );
        return this;
    }

    /**
     * Sets the integral gain constant of the PIDF controller on the Spark Max.
     * @param gain The integral gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setI(double gain, int slotId) {
        addStep(
            pidController -> pidController.setI(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getI(slotId), gain, RevConfigRegistry.EPSILON),
            "I Gain (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Configure the maximum I accumulator of the PID controller. This value is used to constrain the
     * I accumulator to help manage integral wind-up
     * @param iMaxAccum The max value to constrain the I accumulator to.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setIMaxAccum(double iMaxAccum, int slotId) {
        addStep(
            pidController -> pidController.setIMaxAccum(iMaxAccum, slotId),
            pidController -> Math2.epsilonEquals(pidController.getIMaxAccum(slotId), iMaxAccum, RevConfigRegistry.EPSILON),
            "I Max Accumulator (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Sets the IZone range of the PIDF controller on the Spark Max. This value specifies the
     * range the error must be within for the integral constant to take effect.
     * @param iZone The I zone value, must be positive. Set to {@code 0.0} to disable.
     */
    public SparkPIDControllerConfig setIZone(double iZone) {
        addStep(
            pidController -> pidController.setIZone(iZone),
            pidController -> Math2.epsilonEquals(pidController.getIZone(), iZone, RevConfigRegistry.EPSILON),
            "I Zone"
        );
        return this;
    }

    /**
     * Sets the IZone range of the PIDF controller on the Spark Max. This value specifies the
     * range the error must be within for the integral constant to take effect.
     * @param iZone The I zone value, must be positive. Set to {@code 0.0} to disable.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setIZone(double iZone, int slotId) {
        addStep(
            pidController -> pidController.setIZone(iZone, slotId),
            pidController -> Math2.epsilonEquals(pidController.getIZone(slotId), iZone, RevConfigRegistry.EPSILON),
            "I Zone (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Sets the min amd max output for the closed loop mode.
     * @param min Reverse power minimum to allow the controller to output.
     * @param max Forward power maximum to allow the controller to output.
     */
    public SparkPIDControllerConfig setOutputRange(double min, double max) {
        addStep(
            pidController -> pidController.setOutputRange(min, max),
            pidController ->
                Math2.epsilonEquals(pidController.getOutputMin(), min, RevConfigRegistry.EPSILON) &&
                Math2.epsilonEquals(pidController.getOutputMax(), max, RevConfigRegistry.EPSILON),
            "Output Range"
        );
        return this;
    }

    /**
     * Sets the min amd max output for the closed loop mode.
     * @param min Reverse power minimum to allow the controller to output.
     * @param max Forward power maximum to allow the controller to output.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setOutputRange(double min, double max, int slotId) {
        addStep(
            pidController -> pidController.setOutputRange(min, max, slotId),
            pidController ->
                Math2.epsilonEquals(pidController.getOutputMin(slotId), min, RevConfigRegistry.EPSILON) &&
                Math2.epsilonEquals(pidController.getOutputMax(slotId), max, RevConfigRegistry.EPSILON),
            "Output Range (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Sets the proportional gain constant of the PIDF controller on the Spark Max.
     * @param gain The proportional gain value, must be positive.
     */
    public SparkPIDControllerConfig setP(double gain) {
        addStep(
            pidController -> pidController.setP(gain),
            pidController -> Math2.epsilonEquals(pidController.getP(), gain, RevConfigRegistry.EPSILON),
            "P Gain"
        );
        return this;
    }

    /**
     * Sets the proportional gain constant of the PIDF controller on the Spark Max.
     * @param gain The proportional gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setP(double gain, int slotId) {
        addStep(
            pidController -> pidController.setP(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getP(slotId), gain, RevConfigRegistry.EPSILON),
            "P Gain (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Enables or disables PID Wrapping for position closed loop control.
     * @param enable Whether position PID wrapping should be enabled.
     */
    public SparkPIDControllerConfig setPositionPIDWrappingEnabled(boolean enable) {
        addStep(
            pidController -> pidController.setPositionPIDWrappingEnabled(enable),
            pidController -> pidController.getPositionPIDWrappingEnabled() == enable,
            "Position PID Wrapping Enabled"
        );
        return this;
    }

    /**
     * Sets the maximum input value for PID Wrapping with position closed loop control.
     * @param max The value of max input for the position.
     */
    public SparkPIDControllerConfig setPositionPIDWrappingMaxInput(double max) {
        addStep(
            pidController -> pidController.setPositionPIDWrappingMaxInput(max),
            pidController -> Math2.epsilonEquals(pidController.getPositionPIDWrappingMaxInput(), max, RevConfigRegistry.EPSILON),
            "Position PID Wrapping Max Input"
        );
        return this;
    }

    /**
     * Sets the minimum input value for PID Wrapping with position closed loop control.
     * @param min The value of min input for the position.
     */
    public SparkPIDControllerConfig setPositionPIDWrappingMinInput(double min) {
        addStep(
            pidController -> pidController.setPositionPIDWrappingMinInput(min),
            pidController -> Math2.epsilonEquals(pidController.getPositionPIDWrappingMinInput(), min, RevConfigRegistry.EPSILON),
            "Position PID Wrapping Min Input"
        );
        return this;
    }

    /**
     * Sets the minimum and maximum input values for PID Wrapping with position closed loop control.
     * @param min The value of min input for the position.
     * @param max The value of max input for the position.
     */
    public SparkPIDControllerConfig setPositionPIDWrappingInputLimits(double min, double max) {
        setPositionPIDWrappingMinInput(min);
        setPositionPIDWrappingMaxInput(max);
        return this;
    }

    /**
     * Sets PID gains on the Spark Max.
     * @param pGain The proportional gain value, must be positive.
     * @param iGain The integral gain value, must be positive.
     * @param dGain The derivative gain value, must be positive.
     */
    public SparkPIDControllerConfig setPID(double pGain, double iGain, double dGain) {
        setP(pGain);
        setI(iGain);
        setD(dGain);
        return this;
    }

    /**
     * Sets PID gains on the Spark Max.
     * @param pGain The proportional gain value, must be positive.
     * @param iGain The integral gain value, must be positive.
     * @param dGain The derivative gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setPID(double pGain, double iGain, double dGain, int slotId) {
        setP(pGain, slotId);
        setI(iGain, slotId);
        setD(dGain, slotId);
        return this;
    }

    /**
     * Sets PIDF gains on the Spark Max.
     * @param pidConfig The PID config object to apply.
     */
    public SparkPIDControllerConfig setPID(PIDConfig pidfConfig) {
        setPID(pidfConfig.p(), pidfConfig.i(), pidfConfig.d());
        setIZone(pidfConfig.iZone());
        return this;
    }

    /**
     * Sets PIDF gains on the Spark Max.
     * @param pidConfig The PID config object to apply.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setPID(PIDConfig pidfConfig, int slotId) {
        setPID(pidfConfig.p(), pidfConfig.i(), pidfConfig.d(), slotId);
        setIZone(pidfConfig.iZone(), slotId);
        return this;
    }

    /**
     * Sets PIDF gains on the Spark Max.
     * @param pGain The proportional gain value, must be positive.
     * @param iGain The integral gain value, must be positive.
     * @param dGain The derivative gain value, must be positive.
     * @param ffGain The feed-forward gain value, must be positive.
     */
    public SparkPIDControllerConfig setPIDF(double pGain, double iGain, double dGain, double ffGain) {
        setP(pGain);
        setI(iGain);
        setD(dGain);
        setFF(ffGain);
        return this;
    }

    /**
     * Sets PIDF gains on the Spark Max.
     * @param pGain The proportional gain value, must be positive.
     * @param iGain The integral gain value, must be positive.
     * @param dGain The derivative gain value, must be positive.
     * @param ffGain The feed-forward gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setPIDF(double pGain, double iGain, double dGain, double ffGain, int slotId) {
        setP(pGain, slotId);
        setI(iGain, slotId);
        setD(dGain, slotId);
        setFF(ffGain, slotId);
        return this;
    }

    /**
     * Configure the acceleration strategy used to control acceleration on the motor.
     * @param accelStrategy The acceleration strategy to use for the automatically generated motion profile.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setSmartMotionAccelStrategy(AccelStrategy accelStrategy, int slotId) {
        addStep(
            pidController -> pidController.setSmartMotionAccelStrategy(accelStrategy, slotId),
            pidController -> pidController.getSmartMotionAccelStrategy(slotId).equals(accelStrategy),
            "Smart Motion Acceleration Strategy (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Configure the allowed closed loop error of SmartMotion mode. This value is how much deviation
     * from your setpoint is tolerated and is useful in preventing oscillation around your setpoint.
     * @param allowedErr The allowed deviation for your setpoint vs actual position in rotations.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setSmartMotionAllowedClosedLoopError(double allowedErr, int slotId) {
        addStep(
            pidController -> pidController.setSmartMotionAllowedClosedLoopError(allowedErr, slotId),
            pidController ->
                Math2.epsilonEquals(pidController.getSmartMotionAllowedClosedLoopError(slotId), allowedErr, RevConfigRegistry.EPSILON),
            "Smart Motion Allowed Closed Loop Error (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Configure the maximum acceleration of the SmartMotion mode. This is the acceleration that the
     * motor velocity will increase at until the max velocity is reached
     * @param maxAccel The maximum acceleration for the motion profile in RPM per second.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setSmartMotionMaxAccel(double maxAccel, int slotId) {
        addStep(
            pidController -> pidController.setSmartMotionMaxAccel(maxAccel, slotId),
            pidController -> Math2.epsilonEquals(pidController.getSmartMotionMaxAccel(slotId), maxAccel, RevConfigRegistry.EPSILON),
            "Smart Motion Max Acceleration (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Configure the maximum velocity of the SmartMotion mode. This is the velocity that is reached in
     * the middle of the profile and is what the motor should spend most of its time at.
     * @param maxVel The maximum cruise velocity for the motion profile in RPM.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setSmartMotionMaxVelocity(double maxVel, int slotId) {
        addStep(
            pidController -> pidController.setSmartMotionMaxVelocity(maxVel, slotId),
            pidController -> Math2.epsilonEquals(pidController.getSmartMotionMaxVelocity(slotId), maxVel, RevConfigRegistry.EPSILON),
            "Smart Motion Max Velocity (Slot " + slotId + ")"
        );
        return this;
    }

    /**
     * Configure the minimum velocity of the SmartMotion mode. Any requested velocities below this value will be set to {@code 0.0}.
     * @param minVel The minimum velocity for the motion profile in RPM.
     * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
     */
    public SparkPIDControllerConfig setSmartMotionMinOutputVelocity(double minVel, int slotId) {
        addStep(
            pidController -> pidController.setSmartMotionMinOutputVelocity(minVel, slotId),
            pidController -> Math2.epsilonEquals(pidController.getSmartMotionMinOutputVelocity(slotId), minVel, RevConfigRegistry.EPSILON),
            "Smart Motion Min Velocity (Slot " + slotId + ")"
        );
        return this;
    }
}
