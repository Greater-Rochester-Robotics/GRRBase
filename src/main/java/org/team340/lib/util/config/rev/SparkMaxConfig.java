package org.team340.lib.util.config.rev;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.RobotBase;
import org.team340.lib.util.Math2;

/**
 * Config builder for {@link CANSparkMax}.
 */
public final class SparkMaxConfig extends RevConfigBase<CANSparkMax> {
    
    /**
     * Creates an empty config.
     */
    public SparkMaxConfig(){}

    /**
     * Creates a config that copies the config steps from the base provided.
     * @param base The config to copy the steps from.
     */
    private SparkMaxConfig(RevConfigBase<CANSparkMax> base){
        super(base);
    }

    /**
     * Clones this config
     */
    public SparkMaxConfig clone(){
        return new SparkMaxConfig(this);
    }
    private static final double FACTORY_DEFAULTS_SLEEP = 50.0;

    /**
     * Applies the config.
     * @param sparkMax The Spark Max to apply the config to.
     */
    public void apply(CANSparkMax sparkMax) {
        addStep(
            sm -> {
                RevConfigUtils.burnFlashSleep();
                return sm.burnFlash();
            },
            "Burn Flash"
        );
        super.applySteps(sparkMax, "Spark Max (ID " + sparkMax.getDeviceId() + ")");
    }

    /**
     * Spark Max CAN status frames.
     * @see https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
     */
    public static enum Frame {
        /**
         * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-0-default-rate-10ms">Periodic Status 0</a></h2>
         *
         * <i>Default rate: {@code 10ms}</i>
         *
         * <h3> Frame content: </h3>
         *
         * <table>
         *   <tr>
         *     <th> Available Data </th>
         *     <th> Description </th>
         *   </tr>
         *   <tr>
         *     <td> Applied **** Output </td>
         *     <td> The actual value sent to the motors from the motor controller. This value is also used by any follower controllers to set their output. </td>
         *   </tr>
         *   <tr>
         *     <td> Faults </td>
         *     <td> Each bit represents a different fault on the controller. These fault bits clear automatically when the fault goes away. </td>
         *   </tr>
         *   <tr>
         *     <td> Sticky Faults </td>
         *     <td> The same as the Faults field, however the bits do not reset until a power cycle or a 'Clear Faults' command is sent. </td>
         *   </tr>
         *   <tr>
         *     <td> Is Follower </td>
         *     <td> A single bit that is true if the controller is configured to follow another controller. </td>
         *   </tr>
         * </table>
         */
        S0(PeriodicFrame.kStatus0),
        /**
         * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-1-default-rate-20ms">Periodic Status 1</a></h2>
         *
         * <i>Default rate: {@code 20ms}</i>
         *
         * <h3> Frame content: </h3>
         *
         * <table>
         *   <tr>
         *     <th> Available Data </th>
         *     <th> Description </th>
         *   </tr>
         *   <tr>
         *     <th> Motor Velocity </th>
         *     <th> 32-bit IEEE floating-point representation of the motor velocity in RPM using the selected sensor. </th>
         *   </tr>
         *   <tr>
         *     <th> Motor Temperature </th>
         *     <th> 8-bit unsigned value representing motor temperature in °C for the NEO Brushless Motor. </th>
         *   </tr>
         *   <tr>
         *     <th> Motor Voltage </th>
         *     <th> 12-bit fixed-point value that is converted to a floating point voltage value (in Volts) by the roboRIO SDK. This is the input voltage to the controller. </th>
         *   </tr>
         *   <tr>
         *     <th> Motor Current </th>
         *     <th> 12-bit fixed-point value that is converted to a floating point current value (in Amps) by the roboRIO SDK. This is the raw phase current of the motor. </th>
         *   </tr>
         * </table>
         */
        S1(PeriodicFrame.kStatus1),
        /**
         * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-2-default-rate-20ms">Periodic Status 2</a></h2>
         *
         * <i>Default rate: {@code 20ms}</i>
         *
         * <h3> Frame content: </h3>
         *
         * <table>
         *   <tr>
         *     <th> Available Data </th>
         *     <th> Description </th>
         *   </tr>
         *   <tr>
         *     <th> Motor Position </th>
         *     <th> 32-bit IEEE floating-point representation of the motor position in rotations. </th>
         *   </tr>
         * </table>
         */
        S2(PeriodicFrame.kStatus2),
        /**
         * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-3-default-rate-50ms">Periodic Status 3</a></h2>
         *
         * <i>Default rate: {@code 50ms}</i>
         *
         * <h3> Frame content: </h3>
         *
         * <table>
         *   <tr>
         *     <th> Available Data </th>
         *     <th> Description </th>
         *   </tr>
         *   <tr>
         *     <th> Analog Sensor Voltage </th>
         *     <th> 10-bit fixed-point value that is converted to a floating point voltage value (in Volts) by the roboRIO SDK. This is the voltage being output by the analog sensor. </th>
         *   </tr>
         *   <tr>
         *     <th> Analog Sensor Velocity </th>
         *     <th> 22-bit fixed-point value that is converted to a floating point voltage value (in RPM) by the roboRIO SDK. This is the velocity reported by the analog sensor. </th>
         *   </tr>
         *   <tr>
         *     <th> Analog Sensor Position </th>
         *     <th> 32-bit IEEE floating-point representation of the velocity in RPM reported by the analog sensor. </th>
         *   </tr>
         * </table>
         */
        S3(PeriodicFrame.kStatus3),
        /**
         * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-4-default-rate-20ms">Periodic Status 4</a></h2>
         *
         * <i>Default rate: {@code 20ms}</i>
         *
         * <h3> Frame content: </h3>
         *
         * <table>
         *   <tr>
         *     <th> Available Data </th>
         *     <th> Description </th>
         *   </tr>
         *   <tr>
         *     <th> Alternate Encoder Velocity </th>
         *     <th> 32-bit IEEE floating-point representation of the velocity in RPM of the alternate encoder. </th>
         *   </tr>
         *   <tr>
         *     <th> Alternate Encoder Position </th>
         *     <th> 32-bit IEEE floating-point representation of the position in rotations of the alternate encoder. </th>
         *   </tr>
         * </table>
         */
        S4(PeriodicFrame.kStatus4),
        /**
         * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-5-default-rate-200ms">Periodic Status 5</a></h2>
         *
         * <i>Default rate: {@code 200ms}</i>
         *
         * <h3> Frame content: </h3>
         *
         * <table>
         *   <tr>
         *     <th> Available Data </th>
         *     <th> Description </th>
         *   </tr>
         *   <tr>
         *     <th> Duty Cycle Absolute Encoder Position </th>
         *     <th> 32-bit IEEE floating-point representation of the position of the duty cycle absolute encoder. </th>
         *   </tr>
         *   <tr>
         *     <th> Duty Cycle Absolute Encoder Absolute Angle </th>
         *     <th> 16-bit integer representation of the absolute angle of the duty cycle absolute encoder. </th>
         *   </tr>
         * </table>
         */
        S5(PeriodicFrame.kStatus5),
        /**
         * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-6-default-rate-200ms">Periodic Status 6</a></h2>
         *
         * <i>Default rate: {@code 200ms}</i>
         *
         * <h3> Frame content: </h3>
         *
         * <table>
         *   <tr>
         *     <th> Available Data </th>
         *     <th> Description </th>
         *   </tr>
         *   <tr>
         *     <th> Duty Cycle Absolute Encoder Velocity </th>
         *     <th> 32-bit IEEE floating-point representation of the velocity in RPM of the duty cycle absolute encoder. </th>
         *   </tr>
         *   <tr>
         *     <th> Duty Cycle Absolute Encoder Frequency </th>
         *     <th> 16-bit unsigned integer representation of the frequency at which the duty cycle signal is being sent. </th>
         *   </tr>
         * </table>
         */
        S6(PeriodicFrame.kStatus6);

        private final PeriodicFrame frame;

        private Frame(PeriodicFrame frame) {
            this.frame = frame;
        }

        public PeriodicFrame frame() {
            return frame;
        }
    }

    /**
     * Clears all sticky faults.
     */
    public SparkMaxConfig clearFaults() {
        addStep(sparkMax -> sparkMax.clearFaults(), "Clear Faults");
        return this;
    }

    /**
     * Disables the voltage compensation setting for all modes.
     */
    public SparkMaxConfig disableVoltageCompensation() {
        addStep(
            sparkMax -> sparkMax.disableVoltageCompensation(),
            sparkMax -> Math2.epsilonEquals(sparkMax.getVoltageCompensationNominalVoltage(), 0.0, RevConfigUtils.EPSILON),
            "Disable Voltage Compensation"
        );
        return this;
    }

    /**
     * Enables soft limits.
     * @param direction The direction of motion to restrict.
     * @param enable Set {@code true} to enable soft limits.
     */
    public SparkMaxConfig enableSoftLimit(CANSparkMax.SoftLimitDirection direction, boolean enable) {
        addStep(
            sparkMax -> sparkMax.enableSoftLimit(direction, enable),
            sparkMax -> sparkMax.isSoftLimitEnabled(direction) == enable,
            "Enable Soft Limit"
        );
        return this;
    }

    /**
     * Sets the voltage compensation setting for all modes on the Spark Max and enables voltage compensation.
     * @param nominalVoltage Nominal voltage to compensate output to.
     */
    public SparkMaxConfig enableVoltageCompensation(double nominalVoltage) {
        addStep(
            sparkMax -> sparkMax.enableVoltageCompensation(nominalVoltage),
            sparkMax -> Math2.epsilonEquals(sparkMax.getVoltageCompensationNominalVoltage(), nominalVoltage, RevConfigUtils.EPSILON),
            "Enable Voltage Compensation"
        );
        return this;
    }

    /**
     * Causes this controller's output to mirror the provided leader.
     * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     * The motor will spin in the same direction as the leader. This can be changed by passing a {@code true} constant after the leader parameter.
     * @param leader The motor controller to follow.
     */
    public SparkMaxConfig follow(CANSparkMax leader) {
        addStep(sparkMax -> sparkMax.follow(leader), sparkMax -> sparkMax.isFollower(), false, "Follow");
        return this;
    }

    /**
     * Causes this controller's output to mirror the provided leader.
     * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     * @param leader The motor controller to follow.
     * @param invert Set the follower to output opposite of the leader.
     */
    public SparkMaxConfig follow(CANSparkMax leader, boolean invert) {
        addStep(sparkMax -> sparkMax.follow(leader, invert), sparkMax -> sparkMax.isFollower(), false, "Follow");
        return this;
    }

    /**
     * Causes this controller's output to mirror the provided leader.
     * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     * The motor will spin in the same direction as the leader. This can be changed by passing a {@code true} constant after the deviceID parameter.
     * @param leader The type of motor controller to follow (Talon SRX, SPARK MAX, etc.).
     * @param deviceId The CAN ID of the device to follow.
     */
    public SparkMaxConfig follow(CANSparkMax.ExternalFollower leader, int deviceId) {
        addStep(sparkMax -> sparkMax.follow(leader, deviceId), sparkMax -> sparkMax.isFollower(), false, "Follow");
        return this;
    }

    /**
     * Causes this controller's output to mirror the provided leader.
     * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
     * @param leader The type of motor controller to follow (Talon SRX, SPARK MAX, etc.).
     * @param deviceId The CAN ID of the device to follow.
     * @param invert Set the follower to output opposite of the leader.
     */
    public SparkMaxConfig follow(CANSparkMax.ExternalFollower leader, int deviceId, boolean invert) {
        addStep(sparkMax -> sparkMax.follow(leader, deviceId, invert), sparkMax -> sparkMax.isFollower(), false, "Follow");
        return this;
    }

    /**
     * Sets timeout for sending CAN messages with {@code setParameter*()} and {@code getParameter*()}
     *  calls.These calls will block for up to this amount of time before returning a timeout error.
     * A timeout of {@code 0.0} will make the {@code setParameter*()} calls non-blocking, and instead
     * will check the response in a separate thread. With this configuration, any error messages will
     * appear on the driver station but will not be returned by the {@code getLastError()} call.
     * @param milliseconds The timeout in milliseconds.
     */
    public SparkMaxConfig setCANTimeout(int milliseconds) {
        addStep(sparkMax -> sparkMax.setCANTimeout(milliseconds), "CAN Timeout");
        return this;
    }

    /**
     * Sets the ramp rate for closed loop control modes.
     * This is the maximum rate at which the motor controller's output is allowed to change.
     * @param rate Time in seconds to go from {@code 0.0} to full throttle.
     */
    public SparkMaxConfig setClosedLoopRampRate(double rate) {
        addStep(
            sparkMax -> sparkMax.setClosedLoopRampRate(rate),
            sparkMax -> Math2.epsilonEquals(sparkMax.getClosedLoopRampRate(), rate, RevConfigUtils.EPSILON),
            "Closed Loop Ramp Rate"
        );
        return this;
    }

    /**
     * Sets the idle mode setting for the SPARK MAX.
     * @param mode Idle mode (coast or brake).
     */
    public SparkMaxConfig setIdleMode(CANSparkMax.IdleMode mode) {
        addStep(sparkMax -> sparkMax.setIdleMode(mode), sparkMax -> sparkMax.getIdleMode().equals(mode), "Idle Mode");
        return this;
    }

    /**
     * Common interface for inverting direction of a speed controller.
     * This call has no effect if the controller is a follower. To invert a follower, see the {@code follow()} method.
     * @param isInverted The state of inversion, true is inverted.
     */
    public SparkMaxConfig setInverted(boolean isInverted) {
        addStep(
            sparkMax -> {
                sparkMax.setInverted(isInverted);
                return REVLibError.kOk;
            },
            sparkMax -> sparkMax.getInverted() == isInverted,
            "Inverted"
        );
        return this;
    }

    /**
     * Sets the ramp rate for open loop control modes.
     * This is the maximum rate at which the motor controller's output is allowed to change.
     * @param rate Time in seconds to go from {@code 0.0} to full throttle.
     */
    public SparkMaxConfig setOpenLoopRampRate(double rate) {
        addStep(
            sparkMax -> sparkMax.setOpenLoopRampRate(rate),
            sparkMax -> Math2.epsilonEquals(sparkMax.getOpenLoopRampRate(), rate, RevConfigUtils.EPSILON),
            "Open Loop Ramp Rate"
        );
        return this;
    }

    /**
     * Sets the rate of transmission for a periodic frame.
     * Each motor controller sends back status frames with different data at set rates. Use this function to change the default rates.
     * These values are not stored in the flash after calling {@code burnFlash()} and is reset on powerup.
     * @param frame The status frame to configure.
     * @param periodMs The rate the controller sends the frame in milliseconds.
     */
    public SparkMaxConfig setPeriodicFramePeriod(Frame frame, int periodMs) {
        addStep(sparkMax -> sparkMax.setPeriodicFramePeriod(frame.frame, periodMs), "Periodic Frame Status " + frame.ordinal());
        return this;
    }

    /**
     * Sets the secondary current limit in amps.
     * The motor controller will disable the output of the controller briefly if the
     * current limit is exceeded to reduce the current. This limit is a simplified
     * 'on/off' controller. This limit is enabled by default but is set higher than
     * the default Smart Current Limit. The time the controller is off after the current
     * limit is reached is determined by the parameter limitCycles, which is the number
     * of PWM cycles (20kHz). The recommended value is the default of {@code 0.0} which is
     * the minimum time and is part of a PWM cycle from when the over current is detected.
     * This allows the controller to regulate the current close to the limit value.
     * @param limit The current limit in amps.
     */
    public SparkMaxConfig setSecondaryCurrentLimit(double limit) {
        addStep(sparkMax -> sparkMax.setSecondaryCurrentLimit(limit), "Secondary Current Limit");
        return this;
    }

    /**
     * Sets the secondary current limit in amps.
     * The motor controller will disable the output of the controller briefly if the
     * current limit is exceeded to reduce the current. This limit is a simplified
     * 'on/off' controller. This limit is enabled by default but is set higher than
     * the default Smart Current Limit. The time the controller is off after the current
     * limit is reached is determined by the parameter limitCycles, which is the number
     * of PWM cycles (20kHz). The recommended value is the default of {@code 0.0} which is
     * the minimum time and is part of a PWM cycle from when the over current is detected.
     * This allows the controller to regulate the current close to the limit value.
     * @param limit The current limit in amps.
     * @param chopCycles The number of additional PWM cycles to turn the driver off after overcurrent is detected.
     */
    public SparkMaxConfig setSecondaryCurrentLimit(double limit, int chopCycles) {
        addStep(sparkMax -> sparkMax.setSecondaryCurrentLimit(limit, chopCycles), "Secondary Current Limit");
        return this;
    }

    /**
     * Sets the current limit in amps.
     * The motor controller will reduce the controller voltage output to avoid surpassing
     * this limit. This limit is enabled by default and used for brushless only. This limit
     * is highly recommended when using the NEO brushless motor. The NEO Brushless Motor
     * has a low internal resistance, which can mean large current spikes that could be
     * enough to cause damage to the motor and controller. This current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and controller
     * operating in a safe region.
     * @param limit The current limit in amps.
     */
    public SparkMaxConfig setSmartCurrentLimit(int limit) {
        addStep(sparkMax -> sparkMax.setSmartCurrentLimit(limit), "Smart Current Limit");
        return this;
    }

    /**
     * Sets the current limit in amps.
     * The motor controller will reduce the controller voltage output to avoid surpassing
     * this limit. This limit is enabled by default and used for brushless only. This limit
     * is highly recommended when using the NEO brushless motor. The NEO Brushless Motor
     * has a low internal resistance, which can mean large current spikes that could be
     * enough to cause damage to the motor and controller. This current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and controller
     * operating in a safe region. The controller can also limit the current based on the RPM
     * of the motor in a linear fashion to help with controllability in closed loop control.
     * For a response that is linear the entire RPM range leave limit RPM at {@code 0.0}.
     * @param stallLimit The current limit in amps at {@code 0.0} RPM.
     * @param freeLimit The current limit at free speed ({@code 5700} RPM for NEO).
     */
    public SparkMaxConfig setSmartCurrentLimit(int stallLimit, int freeLimit) {
        addStep(sparkMax -> sparkMax.setSmartCurrentLimit(stallLimit, freeLimit), "Smart Current Limit");
        return this;
    }

    /**
     * Sets the current limit in amps.
     * The motor controller will reduce the controller voltage output to avoid surpassing
     * this limit. This limit is enabled by default and used for brushless only. This limit
     * is highly recommended when using the NEO brushless motor. The NEO Brushless Motor
     * has a low internal resistance, which can mean large current spikes that could be
     * enough to cause damage to the motor and controller. This current limit provides a
     * smarter strategy to deal with high current draws and keep the motor and controller
     * operating in a safe region. The controller can also limit the current based on the RPM
     * of the motor in a linear fashion to help with controllability in closed loop control.
     * For a response that is linear the entire RPM range leave limit RPM at {@code 0.0}.
     * @param stallLimit The current limit in amps at {@code 0.0} RPM.
     * @param freeLimit The current limit at free speed ({@code 5700} RPM for NEO).
     * @param limitRPM RPM less than this value will be set to the stallLimit, RPM values greater than limitRPM will scale linearly to freeLimit.
     */
    public SparkMaxConfig setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
        addStep(sparkMax -> sparkMax.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM), "Smart Current Limit");
        return this;
    }

    /**
     * Sets the soft limit based on position. The default unit is rotations, but will match
     * the unit scaling set by the user. Note that this value is not scaled internally so
     * care must be taken to make sure these units match the desired conversion.
     * @param direction The direction of motion to restrict.
     * @param limit Position soft limit of the controller.
     */
    public SparkMaxConfig setSoftLimit(CANSparkMax.SoftLimitDirection direction, double limit) {
        addStep(
            sparkMax -> sparkMax.setSoftLimit(direction, (float) limit),
            sparkMax -> Math2.epsilonEquals(sparkMax.getSoftLimit(direction), limit, RevConfigUtils.EPSILON),
            "Soft Limit"
        );
        return this;
    }

    /**
     * Restores motor controller parameters to factory defaults.
     * <p><b>This option should be declared first, to ensure all other configuration options aren't overwritten.</b></p>
     */
    public SparkMaxConfig restoreFactoryDefaults() {
        addStep(
            sparkMax -> {
                REVLibError res = sparkMax.restoreFactoryDefaults();

                if (!RobotBase.isSimulation()) {
                    try {
                        Thread.sleep((long) FACTORY_DEFAULTS_SLEEP);
                    } catch (Exception e) {}
                }

                return res;
            },
            "Restore Factory Defaults"
        );
        return this;
    }

    /**
     * Restores motor controller parameters to factory defaults.
     * @param persist If {@code true}, burn the flash with the factory default parameters.
     * <p><b>This option should be declared first, to ensure all other configuration options aren't overwritten.</b></p>
     */
    public SparkMaxConfig restoreFactoryDefaults(boolean persist) {
        addStep(
            sparkMax -> {
                REVLibError res = sparkMax.restoreFactoryDefaults(persist);

                if (!RobotBase.isSimulation()) {
                    try {
                        Thread.sleep((long) FACTORY_DEFAULTS_SLEEP);
                    } catch (Exception e) {}
                }

                return res;
            },
            "Restore Factory Defaults"
        );
        return this;
    }
}
