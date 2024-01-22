package org.team340.lib.util.config.rev;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import org.team340.lib.util.Math2;

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
        addStep(
            pc -> {
                RevConfigUtils.burnFlashSleep();
                return sparkMax.burnFlash();
            },
            "Burn Flash"
        );
        super.applySteps(pidController, "Spark Max (ID " + sparkMax.getDeviceId() + ") PID Controller");
    }

    /**
     * Applies the config.
     * @param sparkFlex The Spark Flex to apply the config to.
     * @param pidController The PID controller.
     */
    public void apply(CANSparkFlex sparkFlex, SparkPIDController pidController) {
        addStep(
            pc -> {
                RevConfigUtils.burnFlashSleep();
                return sparkFlex.burnFlash();
            },
            "Burn Flash"
        );
        super.applySteps(pidController, "Spark Flex (ID " + sparkFlex.getDeviceId() + ") PID Controller");
    }

    /**
     * Sets the derivative gain constant of the PIDF controller on the Spark Max.
     * @param gain The derivative gain value, must be positive.
     */
    public SparkPIDControllerConfig setD(double gain) {
        addStep(
            pidController -> pidController.setD(gain),
            pidController -> Math2.epsilonEquals(pidController.getD(), gain, RevConfigUtils.EPSILON),
            "D Gain"
        );
        return this;
    }

    /**
     * Sets the derivative gain constant of the PIDF controller on the Spark Max.
     * @param gain The derivative gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setD(double gain, int slotId) {
        addStep(
            pidController -> pidController.setD(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getD(slotId), gain, RevConfigUtils.EPSILON),
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
            pidController -> Math2.epsilonEquals(pidController.getDFilter(0), gain, RevConfigUtils.EPSILON),
            "D Filter"
        );
        return this;
    }

    /**
     * Sets the the derivative filter constant of the PIDF controller on the Spark Max.
     * @param gain The derivative filter value, must be a positive number between {@code 0.0} and {@code 1.0}.
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setDFilter(double gain, int slotId) {
        addStep(
            pidController -> pidController.setDFilter(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getDFilter(slotId), gain, RevConfigUtils.EPSILON),
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
            pidController -> Math2.epsilonEquals(pidController.getFF(), gain, RevConfigUtils.EPSILON),
            "FF Gain"
        );
        return this;
    }

    /**
     * Sets the  feed-forward gain constant of the PIDF controller on the Spark Max.
     * @param gain The  feed-forward gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setFF(double gain, int slotId) {
        addStep(
            pidController -> pidController.setFF(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getFF(slotId), gain, RevConfigUtils.EPSILON),
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
            pidController -> Math2.epsilonEquals(pidController.getI(), gain, RevConfigUtils.EPSILON),
            "I Gain"
        );
        return this;
    }

    /**
     * Sets the integral gain constant of the PIDF controller on the Spark Max.
     * @param gain The integral gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setI(double gain, int slotId) {
        addStep(
            pidController -> pidController.setI(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getI(slotId), gain, RevConfigUtils.EPSILON),
            "I Gain (Slot " + slotId + ")"
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
            pidController -> Math2.epsilonEquals(pidController.getIZone(), iZone, RevConfigUtils.EPSILON),
            "I Zone"
        );
        return this;
    }

    /**
     * Sets the IZone range of the PIDF controller on the Spark Max. This value specifies the
     * range the error must be within for the integral constant to take effect.
     * @param iZone The I zone value, must be positive. Set to {@code 0.0} to disable.
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setIZone(double iZone, int slotId) {
        addStep(
            pidController -> pidController.setIZone(iZone, slotId),
            pidController -> Math2.epsilonEquals(pidController.getIZone(slotId), iZone, RevConfigUtils.EPSILON),
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
                Math2.epsilonEquals(pidController.getOutputMin(), min, RevConfigUtils.EPSILON) &&
                Math2.epsilonEquals(pidController.getOutputMax(), max, RevConfigUtils.EPSILON),
            "Output Range"
        );
        return this;
    }

    /**
     * Sets the min amd max output for the closed loop mode.
     * @param min Reverse power minimum to allow the controller to output.
     * @param max Forward power maximum to allow the controller to output.
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setOutputRange(double min, double max, int slotId) {
        addStep(
            pidController -> pidController.setOutputRange(min, max, slotId),
            pidController ->
                Math2.epsilonEquals(pidController.getOutputMin(slotId), min, RevConfigUtils.EPSILON) &&
                Math2.epsilonEquals(pidController.getOutputMax(slotId), max, RevConfigUtils.EPSILON),
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
            pidController -> Math2.epsilonEquals(pidController.getP(), gain, RevConfigUtils.EPSILON),
            "P Gain"
        );
        return this;
    }

    /**
     * Sets the proportional gain constant of the PIDF controller on the Spark Max.
     * @param gain The proportional gain value, must be positive.
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setP(double gain, int slotId) {
        addStep(
            pidController -> pidController.setP(gain, slotId),
            pidController -> Math2.epsilonEquals(pidController.getP(slotId), gain, RevConfigUtils.EPSILON),
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
            pidController -> Math2.epsilonEquals(pidController.getPositionPIDWrappingMaxInput(), max, RevConfigUtils.EPSILON),
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
            pidController -> Math2.epsilonEquals(pidController.getPositionPIDWrappingMinInput(), min, RevConfigUtils.EPSILON),
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
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setPID(double pGain, double iGain, double dGain, int slotId) {
        setP(pGain, slotId);
        setI(iGain, slotId);
        setD(dGain, slotId);
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
     * @param slotId The gain schedule slot, the value is a number between {@code 0.0} and {@code 3}.
     */
    public SparkPIDControllerConfig setPIDF(double pGain, double iGain, double dGain, double ffGain, int slotId) {
        setP(pGain, slotId);
        setI(iGain, slotId);
        setD(dGain, slotId);
        setFF(ffGain, slotId);
        return this;
    }
}
