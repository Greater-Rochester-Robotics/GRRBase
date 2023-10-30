package org.team340.lib.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.text.Collator;
import java.util.Collection;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.Supplier;

/**
 * Utilities for REVLib interfaces.
 */
public final class RevUtil {

    private static final double EPSILON = 1e-4;
    private static final double BURN_FLASH_SLEEP = 100;
    private static final double CHECK_SLEEP = 25;
    private static final int SET_ITERATIONS = 3;

    private static final Collection<String> setSuccess = new TreeSet<>(SuccessComparator.getInstance());
    private static double period = 0.020;

    private RevUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Spark Max configuration utilities.
     */
    public static final class SparkMax {

        private SparkMax() {
            throw new UnsupportedOperationException("This is a utility class!");
        }

        /**
         * Spark Max CAN status frames.
         * @see https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
         */
        public static enum Frame {
            /**
             * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-0-default-rate-10ms">Periodic Status 0</a></h2>
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
             *
             * <h3> Rates: </h3>
             *
             * <table>
             *   <tr>
             *     <td> <b>Default:</b> 10ms </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Enabled:</b> <code>period / 2</code> </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Disabled:</b> 250ms </td>
             *   </tr>
             * </table>
             */
            S0((int) ((period * 1000.0) / 2.0), 250),
            /**
             * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-1-default-rate-20ms">Periodic Status 1</a></h2>
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
             *
             * <h3> Rates: </h3>
             *
             * <table>
             *   <tr>
             *     <td> <b>Default:</b> 20ms </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Enabled:</b> <code>period</code> </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Disabled:</b> 250ms </td>
             *   </tr>
             * </table>
             */
            S1((int) (period * 1000.0), 250),
            /**
             * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-2-default-rate-20ms">Periodic Status 2</a></h2>
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
             *
             * <h3> Rates: </h3>
             *
             * <table>
             *   <tr>
             *     <td> <b>Default:</b> 20ms </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Enabled:</b> <code>period</code> </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Disabled:</b> 250ms </td>
             *   </tr>
             * </table>
             */
            S2((int) (period * 1000.0), 250),
            /**
             * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-3-default-rate-50ms">Periodic Status 3</a></h2>
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
             *
             * <h3> Rates: </h3>
             *
             * <table>
             *   <tr>
             *     <td> <b>Default:</b> 50ms </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Enabled:</b> <code>period</code> </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Disabled:</b> 10000ms </td>
             *   </tr>
             * </table>
             */
            S3((int) (period * 1000.0), 10000),
            /**
             * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-4-default-rate-20ms">Periodic Status 4</a></h2>
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
             *
             * <h3> Rates: </h3>
             *
             * <table>
             *   <tr>
             *     <td> <b>Default:</b> 20ms </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Enabled:</b> <code>period</code> </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Disabled:</b> 10000ms </td>
             *   </tr>
             * </table>
             */
            S4((int) (period * 1000.0), 10000),
            /**
             * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-5-default-rate-200ms">Periodic Status 5</a></h2>
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
             *
             * <h3> Rates: </h3>
             *
             * <table>
             *   <tr>
             *     <td> <b>Default:</b> 200ms </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Enabled:</b> <code>period</code> </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Disabled:</b> 10000ms </td>
             *   </tr>
             * </table>
             */
            S5((int) (period * 1000.0), 10000),
            /**
             * <h2><a href="https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-6-default-rate-200ms">Periodic Status 6</a></h2>
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
             *
             * <h3> Rates: </h3>
             *
             * <table>
             *   <tr>
             *     <td> <b>Default:</b> 200ms </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Enabled:</b> <code>period</code> </td>
             *   </tr>
             *   <tr>
             *     <td> <b>Disabled:</b> 10000ms </td>
             *   </tr>
             * </table>
             */
            S6((int) (period * 1000.0), 10000);

            private final int flag = 1 << ordinal();
            private final int onPeriodMs;
            private final int offPeriodMs;

            private Frame(int onPeriodMs, int offPeriodMs) {
                this.onPeriodMs = onPeriodMs;
                this.offPeriodMs = offPeriodMs;
            }

            /**
             * @internal
             */
            public int getPeriodMs(int flags) {
                if ((flags & flag) == flag) {
                    return onPeriodMs;
                } else {
                    return offPeriodMs;
                }
            }
        }

        /**
         * Writes all settings to flash.
         * @param sparkMax The Spark Max to configure.
         */
        public static void burnFlash(CANSparkMax sparkMax) {
            if (!RobotBase.isSimulation()) {
                try {
                    Thread.sleep((long) BURN_FLASH_SLEEP);
                } catch (Exception e) {}
            }

            set(sparkMax, () -> sparkMax.burnFlash(), "Burn Flash");
        }

        /**
         * Clears all sticky faults.
         * @param sparkMax The Spark Max to configure.
         */
        public static void clearFaults(CANSparkMax sparkMax) {
            set(sparkMax, () -> sparkMax.clearFaults(), "Clear Faults");
        }

        /**
         * Disables the voltage compensation setting for all modes.
         * @param sparkMax The Spark Max to configure.
         */
        public static void disableVoltageCompensation(CANSparkMax sparkMax) {
            set(
                sparkMax,
                () -> sparkMax.disableVoltageCompensation(),
                () -> Math2.epsilonEquals(sparkMax.getVoltageCompensationNominalVoltage(), 0.0, EPSILON),
                "Disable Voltage Compensation"
            );
        }

        /**
         * Enables soft limits.
         * @param sparkMax The Spark Max to configure.
         * @param direction The direction of motion to restrict.
         * @param enable Set {@code true} to enable soft limits.
         */
        public static void enableSoftLimit(CANSparkMax sparkMax, CANSparkMax.SoftLimitDirection direction, boolean enable) {
            set(
                sparkMax,
                () -> sparkMax.enableSoftLimit(direction, enable),
                () -> sparkMax.isSoftLimitEnabled(direction) == enable,
                "Enable Soft Limit"
            );
        }

        /**
         * Sets the voltage compensation setting for all modes on the Spark Max and enables voltage compensation.
         * @param sparkMax The Spark Max to configure.
         * @param nominalVoltage Nominal voltage to compensate output to.
         */
        public static void enableVoltageCompensation(CANSparkMax sparkMax, double nominalVoltage) {
            set(
                sparkMax,
                () -> sparkMax.enableVoltageCompensation(nominalVoltage),
                () -> Math2.epsilonEquals(sparkMax.getVoltageCompensationNominalVoltage(), nominalVoltage, EPSILON),
                "Enable Voltage Compensation"
            );
        }

        /**
         * Causes this controller's output to mirror the provided leader.
         * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
         * The motor will spin in the same direction as the leader. This can be changed by passing a {@code true} constant after the leader parameter.
         * @param sparkMax The Spark Max to configure.
         * @param leader The motor controller to follow.
         */
        public static void follow(CANSparkMax sparkMax, CANSparkMax leader) {
            set(sparkMax, () -> sparkMax.follow(leader), () -> sparkMax.isFollower(), false, "Follow");
        }

        /**
         * Causes this controller's output to mirror the provided leader.
         * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
         * @param sparkMax The Spark Max to configure.
         * @param leader The motor controller to follow.
         * @param invert Set the follower to output opposite of the leader.
         */
        public static void follow(CANSparkMax sparkMax, CANSparkMax leader, boolean invert) {
            set(sparkMax, () -> sparkMax.follow(leader, invert), () -> sparkMax.isFollower(), false, "Follow");
        }

        /**
         * Causes this controller's output to mirror the provided leader.
         * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
         * The motor will spin in the same direction as the leader. This can be changed by passing a {@code true} constant after the deviceID parameter.
         * @param sparkMax The Spark Max to configure.
         * @param leader The type of motor controller to follow (Talon SRX, SPARK MAX, etc.).
         * @param deviceId The CAN ID of the device to follow.
         */
        public static void follow(CANSparkMax sparkMax, CANSparkMax.ExternalFollower leader, int deviceId) {
            set(sparkMax, () -> sparkMax.follow(leader, deviceId), () -> sparkMax.isFollower(), false, "Follow");
        }

        /**
         * Causes this controller's output to mirror the provided leader.
         * Only voltage output is mirrored. Settings changed on the leader do not affect the follower.
         * @param sparkMax The Spark Max to configure.
         * @param leader The type of motor controller to follow (Talon SRX, SPARK MAX, etc.).
         * @param deviceId The CAN ID of the device to follow.
         * @param invert Set the follower to output opposite of the leader.
         */
        public static void follow(CANSparkMax sparkMax, CANSparkMax.ExternalFollower leader, int deviceId, boolean invert) {
            set(sparkMax, () -> sparkMax.follow(leader, deviceId, invert), () -> sparkMax.isFollower(), false, "Follow");
        }

        /**
         * Sets timeout for sending CAN messages with {@code setParameter*()} and {@code getParameter*()}
         *  calls.These calls will block for up to this amount of time before returning a timeout error.
         * A timeout of {@code 0} will make the {@code setParameter*()} calls non-blocking, and instead
         * will check the response in a separate thread. With this configuration, any error messages will
         * appear on the driver station but will not be returned by the {@code getLastError()} call.
         * @param sparkMax The Spark Max to configure.
         * @param milliseconds The timeout in milliseconds.
         */
        public static void setCANTimeout(CANSparkMax sparkMax, int milliseconds) {
            set(sparkMax, () -> sparkMax.setCANTimeout(milliseconds), "CAN Timeout");
        }

        /**
         * Sets the ramp rate for closed loop control modes.
         * This is the maximum rate at which the motor controller's output is allowed to change.
         * @param sparkMax The Spark Max to configure.
         * @param rate Time in seconds to go from {@code 0} to full throttle.
         */
        public static void setClosedLoopRampRate(CANSparkMax sparkMax, double rate) {
            set(
                sparkMax,
                () -> sparkMax.setClosedLoopRampRate(rate),
                () -> Math2.epsilonEquals(sparkMax.getClosedLoopRampRate(), rate, EPSILON),
                "Closed Loop Ramp Rate"
            );
        }

        /**
         * Sets the idle mode setting for the SPARK MAX.
         * @param sparkMax The Spark Max to configure.
         * @param mode Idle mode (coast or brake).
         */
        public static void setIdleMode(CANSparkMax sparkMax, CANSparkMax.IdleMode mode) {
            set(sparkMax, () -> sparkMax.setIdleMode(mode), () -> sparkMax.getIdleMode().equals(mode), "Idle Mode");
        }

        /**
         * Common interface for inverting direction of a speed controller.
         * This call has no effect if the controller is a follower. To invert a follower, see the {@code follow()} method.
         * @param sparkMax The Spark Max to configure.
         * @param isInverted The state of inversion, true is inverted.
         */
        public static void setInverted(CANSparkMax sparkMax, boolean isInverted) {
            set(
                sparkMax,
                () -> {
                    sparkMax.setInverted(isInverted);
                    return REVLibError.kOk;
                },
                () -> sparkMax.getInverted() == isInverted,
                "Inverted"
            );
        }

        /**
         * Sets the ramp rate for open loop control modes.
         * This is the maximum rate at which the motor controller's output is allowed to change.
         * @param sparkMax The Spark Max to configure.
         * @param rate Time in seconds to go from {@code 0} to full throttle.
         */
        public static void setOpenLoopRampRate(CANSparkMax sparkMax, double rate) {
            set(
                sparkMax,
                () -> sparkMax.setOpenLoopRampRate(rate),
                () -> Math2.epsilonEquals(sparkMax.getOpenLoopRampRate(), rate, EPSILON),
                "Open Loop Ramp Rate"
            );
        }

        /**
         * Sets the secondary current limit in amps.
         * The motor controller will disable the output of the controller briefly if the
         * current limit is exceeded to reduce the current. This limit is a simplified
         * 'on/off' controller. This limit is enabled by default but is set higher than
         * the default Smart Current Limit. The time the controller is off after the current
         * limit is reached is determined by the parameter limitCycles, which is the number
         * of PWM cycles (20kHz). The recommended value is the default of {@code 0} which is
         * the minimum time and is part of a PWM cycle from when the over current is detected.
         * This allows the controller to regulate the current close to the limit value.
         * @param sparkMax The Spark Max to configure.
         * @param limit The current limit in amps.
         */
        public static void setSecondaryCurrentLimit(CANSparkMax sparkMax, double limit) {
            set(sparkMax, () -> sparkMax.setSecondaryCurrentLimit(limit), "Secondary Current Limit");
        }

        /**
         * Sets the secondary current limit in amps.
         * The motor controller will disable the output of the controller briefly if the
         * current limit is exceeded to reduce the current. This limit is a simplified
         * 'on/off' controller. This limit is enabled by default but is set higher than
         * the default Smart Current Limit. The time the controller is off after the current
         * limit is reached is determined by the parameter limitCycles, which is the number
         * of PWM cycles (20kHz). The recommended value is the default of {@code 0} which is
         * the minimum time and is part of a PWM cycle from when the over current is detected.
         * This allows the controller to regulate the current close to the limit value.
         * @param sparkMax The Spark Max to configure.
         * @param limit The current limit in amps.
         * @param chopCycles The number of additional PWM cycles to turn the driver off after overcurrent is detected.
         */
        public static void setSecondaryCurrentLimit(CANSparkMax sparkMax, double limit, int chopCycles) {
            set(sparkMax, () -> sparkMax.setSecondaryCurrentLimit(limit, chopCycles), "Secondary Current Limit");
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
         * @param sparkMax The Spark Max to configure.
         * @param limit The current limit in amps.
         */
        public static void setSmartCurrentLimit(CANSparkMax sparkMax, int limit) {
            set(sparkMax, () -> sparkMax.setSmartCurrentLimit(limit), "Smart Current Limit");
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
         * For a response that is linear the entire RPM range leave limit RPM at {@code 0}.
         * @param sparkMax The Spark Max to configure.
         * @param stallLimit The current limit in amps at {@code 0} RPM.
         * @param freeLimit The current limit at free speed ({@code 5700} RPM for NEO).
         */
        public static void setSmartCurrentLimit(CANSparkMax sparkMax, int stallLimit, int freeLimit) {
            set(sparkMax, () -> sparkMax.setSmartCurrentLimit(stallLimit, freeLimit), "Smart Current Limit");
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
         * For a response that is linear the entire RPM range leave limit RPM at {@code 0}.
         * @param sparkMax The Spark Max to configure.
         * @param stallLimit The current limit in amps at {@code 0} RPM.
         * @param freeLimit The current limit at free speed ({@code 5700} RPM for NEO).
         * @param limitRPM RPM less than this value will be set to the stallLimit, RPM values greater than limitRPM will scale linearly to freeLimit.
         */
        public static void setSmartCurrentLimit(CANSparkMax sparkMax, int stallLimit, int freeLimit, int limitRPM) {
            set(sparkMax, () -> sparkMax.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM), "Smart Current Limit");
        }

        /**
         * Sets the soft limit based on position. The default unit is rotations, but will match
         * the unit scaling set by the user. Note that this value is not scaled internally so
         * care must be taken to make sure these units match the desired conversion.
         * @param sparkMax The Spark Max to configure.
         * @param direction The direction of motion to restrict.
         * @param limit Position soft limit of the controller.
         */
        public static void setSoftLimit(CANSparkMax sparkMax, CANSparkMax.SoftLimitDirection direction, double limit) {
            set(
                sparkMax,
                () -> sparkMax.setSoftLimit(direction, (float) limit),
                () -> Math2.epsilonEquals(sparkMax.getSoftLimit(direction), limit, EPSILON),
                "Soft Limit"
            );
        }

        /**
         * Restores motor controller parameters to factory defaults.
         * @param sparkMax The Spark Max to configure.
         */
        public static void restoreFactoryDefaults(CANSparkMax sparkMax) {
            set(sparkMax, () -> sparkMax.restoreFactoryDefaults(), "Restore Factory Defaults");
        }

        /**
         * Restores motor controller parameters to factory defaults.
         * @param sparkMax The Spark Max to configure.
         * @param persist If {@code true}, burn the flash with the factory default parameters.
         */
        public static void restoreFactoryDefaults(CANSparkMax sparkMax, boolean persist) {
            set(sparkMax, () -> sparkMax.restoreFactoryDefaults(persist), "Restore Factory Defaults");
        }

        /**
         * Sets the rate of transmission for periodic frames.
         * Each motor controller sends back status frames with different data at set rates. Use this function to change the default rates.
         * These values are not stored in the flash after calling {@code burnFlash()} and is reset on powerup.
         * @param sparkMax The Spark Max to configure.
         * @param use Status frames to use.
         */
        public static void setPeriodicFramePeriods(CANSparkMax sparkMax, Frame... use) {
            int flagAggregator = 0;
            for (Frame frame : use) {
                flagAggregator |= frame.flag;
            }

            int flags = flagAggregator;

            set(
                sparkMax,
                () -> sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, Frame.S0.getPeriodMs(flags)),
                "Periodic Frame Status 0"
            );
            set(
                sparkMax,
                () -> sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, Frame.S1.getPeriodMs(flags)),
                "Periodic Frame Status 1"
            );
            set(
                sparkMax,
                () -> sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, Frame.S2.getPeriodMs(flags)),
                "Periodic Frame Status 2"
            );
            set(
                sparkMax,
                () -> sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, Frame.S3.getPeriodMs(flags)),
                "Periodic Frame Status 3"
            );
            set(
                sparkMax,
                () -> sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus4, Frame.S4.getPeriodMs(flags)),
                "Periodic Frame Status 4"
            );
            set(
                sparkMax,
                () -> sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus5, Frame.S5.getPeriodMs(flags)),
                "Periodic Frame Status 5"
            );
            set(
                sparkMax,
                () -> sparkMax.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus6, Frame.S6.getPeriodMs(flags)),
                "Periodic Frame Status 6"
            );
        }
    }

    /**
     * Spark Max absolute encoder configuration utilities.
     */
    public static final class AbsoluteEncoder {

        private AbsoluteEncoder() {
            throw new UnsupportedOperationException("This is a utility class!");
        }

        /**
         * Sets the average sampling depth for an absolute encoder. This is a bit size and should be
         * either {@code 1}, {@code 2}, {@code 4}, {@code 8}, {@code 16}, {@code 32}, {@code 64}, or {@code 128}.
         * @param sparkMax The Spark Max the absolute encoder is attached to.
         * @param absoluteEncoder The absolute encoder to configure.
         * @param depth The average sampling depth of {@code 1}, {@code 2}, {@code 4}, {@code 8}, {@code 16}, {@code 32}, {@code 64}, or {@code 128}.
         */
        public static void setAverageDepth(CANSparkMax sparkMax, com.revrobotics.AbsoluteEncoder absoluteEncoder, int depth) {
            set(
                sparkMax,
                () -> absoluteEncoder.setAverageDepth(depth),
                () -> absoluteEncoder.getAverageDepth() == depth,
                "Absolute Encoder Average Depth"
            );
        }

        /**
         * Sets the phase of the absolute encoder so that it is set to be in phase with the motor itself.
         * @param sparkMax The Spark Max the absolute encoder is attached to.
         * @param absoluteEncoder The absolute encoder to configure.
         * @param inverted The phase of the encoder.
         */
        public static void setInverted(CANSparkMax sparkMax, com.revrobotics.AbsoluteEncoder absoluteEncoder, boolean inverted) {
            set(
                sparkMax,
                () -> absoluteEncoder.setInverted(inverted),
                () -> absoluteEncoder.getInverted() == inverted,
                "Absolute Encoder Inverted"
            );
        }

        /**
         * Sets the conversion factor for position of the encoder. Multiplied
         * by the native output units to give you position.
         * @param sparkMax The Spark Max the absolute encoder is attached to.
         * @param absoluteEncoder The absolute encoder to configure.
         * @param factor The conversion factor to multiply the native units (rotations) by.
         */
        public static void setPositionConversionFactor(
            CANSparkMax sparkMax,
            com.revrobotics.AbsoluteEncoder absoluteEncoder,
            double factor
        ) {
            set(
                sparkMax,
                () -> absoluteEncoder.setPositionConversionFactor(factor),
                () -> Math2.epsilonEquals(absoluteEncoder.getPositionConversionFactor(), factor, EPSILON),
                "Absolute Encoder Position Conversion Factor"
            );
        }

        /**
         * Sets the conversion factor for velocity of the encoder. Multiplied
         * by the native output units to give you velocity.
         * @param sparkMax The Spark Max the absolute encoder is attached to.
         * @param absoluteEncoder The absolute encoder to configure.
         * @param factor The conversion factor to multiply the native units (rotations per minute) by.
         */
        public static void setVelocityConversionFactor(
            CANSparkMax sparkMax,
            com.revrobotics.AbsoluteEncoder absoluteEncoder,
            double factor
        ) {
            set(
                sparkMax,
                () -> absoluteEncoder.setVelocityConversionFactor(factor),
                () -> Math2.epsilonEquals(absoluteEncoder.getVelocityConversionFactor(), factor, EPSILON),
                "Absolute Encoder Velocity Conversion Factor"
            );
        }

        /**
         * Sets the zero offset of an absolute encoder (the position that is reported as zero).
         * The zero offset is specified as the reported position of the encoder in the desired
         * zero position, if the zero offset was set to 0. It is influenced by the absolute
         * encoder's position conversion factor, and whether it is inverted. Always call
         * {@code setConversionFactor()} and {@code setInverted()}
         * before calling this function.
         * @param sparkMax The Spark Max the absolute encoder is attached to.
         * @param absoluteEncoder The absolute encoder to configure.
         * @param offset The zero offset with the position conversion factor applied.
         */
        public static void setZeroOffset(CANSparkMax sparkMax, com.revrobotics.AbsoluteEncoder absoluteEncoder, double offset) {
            set(
                sparkMax,
                () -> absoluteEncoder.setZeroOffset(offset),
                () -> Math2.epsilonEquals(absoluteEncoder.getZeroOffset(), offset, EPSILON),
                "Absolute Encoder Zero Offset"
            );
        }
    }

    /**
     * Spark Max relative encoder configuration utilities.
     */
    public static final class RelativeEncoder {

        private RelativeEncoder() {
            throw new UnsupportedOperationException("This is a utility class!");
        }

        /**
         * Sets the sampling depth of the velocity calculation process for a quadrature or hall sensor
         * encoder. This value sets the number of samples in the average for velocity readings. For a
         * quadrature encoder, this can be any value from {@code 1} to {@code 64} (default). For a hall
         * sensor, it must be either {@code 1}, {@code 2}, {@code 4}, or {@code 8} (default).
         * @param sparkMax The Spark Max the relative encoder is attached to.
         * @param relativeEncoder The relative encoder to configure.
         * @param depth The velocity calculation process's sampling depth.
         */
        public static void setAverageDepth(CANSparkMax sparkMax, com.revrobotics.RelativeEncoder relativeEncoder, int depth) {
            set(
                sparkMax,
                () -> relativeEncoder.setAverageDepth(depth),
                () -> relativeEncoder.getAverageDepth() == depth,
                "Relative Encoder Average Depth"
            );
        }

        /**
         * Sets the phase of the motor feedback sensor so that it is set to be in phase with the motor itself.
         * This only works for quadrature encoders and analog sensors. This will throw an error if the user
         * tries to set the inversion of the hall sensor.
         * @param sparkMax The Spark Max the relative encoder is attached to.
         * @param relativeEncoder The relative encoder to configure.
         * @param inverted The phase of the sensor.
         */
        public static void setInverted(CANSparkMax sparkMax, com.revrobotics.RelativeEncoder relativeEncoder, boolean inverted) {
            set(
                sparkMax,
                () -> relativeEncoder.setInverted(inverted),
                () -> relativeEncoder.getInverted() == inverted,
                "Relative Encoder Inverted"
            );
        }

        /**
         * Sets the position measurement period used to calculate the velocity of a quadrature or hall sensor
         * encoder. For a quadrature encoder, this number may be between {@code 1} and {@code 100} (default). For
         * a hall sensor, this number may be between {@code 8} and {@code 64}. The default for a hall sensor is 32ms.
         * @param sparkMax The Spark Max the relative encoder is attached to.
         * @param relativeEncoder The relative encoder to configure.
         * @param periodMs Measurement period in milliseconds.
         */
        public static void setMeasurementPeriod(CANSparkMax sparkMax, com.revrobotics.RelativeEncoder relativeEncoder, int periodMs) {
            set(
                sparkMax,
                () -> relativeEncoder.setMeasurementPeriod(periodMs),
                () -> relativeEncoder.getMeasurementPeriod() == periodMs,
                "Relative Encoder Measurement Period"
            );
        }

        /**
         * Sets the conversion factor for position of the encoder. Multiplied
         * by the native output units to give you position.
         * @param sparkMax The Spark Max the relative encoder is attached to.
         * @param relativeEncoder The relative encoder to configure.
         * @param factor The conversion factor to multiply the native units by.
         */
        public static void setPositionConversionFactor(
            CANSparkMax sparkMax,
            com.revrobotics.RelativeEncoder relativeEncoder,
            double factor
        ) {
            set(
                sparkMax,
                () -> relativeEncoder.setPositionConversionFactor(factor),
                () -> Math2.epsilonEquals(relativeEncoder.getPositionConversionFactor(), factor, EPSILON),
                "Relative Encoder Position Conversion Factor"
            );
        }

        /**
         * Sets the conversion factor for velocity of the encoder. Multiplied
         * by the native output units to give you velocity.
         * @param sparkMax The Spark Max the relative encoder is attached to.
         * @param relativeEncoder The relative encoder to configure.
         * @param factor The conversion factor to multiply the native units by.
         */
        public static void setVelocityConversionFactor(
            CANSparkMax sparkMax,
            com.revrobotics.RelativeEncoder relativeEncoder,
            double factor
        ) {
            set(
                sparkMax,
                () -> relativeEncoder.setVelocityConversionFactor(factor),
                () -> Math2.epsilonEquals(relativeEncoder.getVelocityConversionFactor(), factor, EPSILON),
                "Relative Encoder Velocity Conversion Factor"
            );
        }
    }

    /**
     * Spark Max analog sensor configuration utilities.
     */
    public static final class AnalogSensor {

        private AnalogSensor() {
            throw new UnsupportedOperationException("This is a utility class!");
        }

        /**
         * Sets the phase of the analog sensor so that it is set to be in phase with the motor itself.
         * @param sparkMax The Spark Max the analog sensor is attached to.
         * @param analogSensor The analog sensor to configure.
         * @param inverted The phase of the sensor.
         */
        public static void setInverted(CANSparkMax sparkMax, SparkMaxAnalogSensor analogSensor, boolean inverted) {
            set(sparkMax, () -> analogSensor.setInverted(inverted), () -> analogSensor.getInverted() == inverted, "Analog Sensor Inverted");
        }

        /**
         * Sets the conversion factor for the position of the analog sensor. By default, revolutions per
         * volt is {@code 1}. Changing the position conversion factor will also change the position units.
         * @param sparkMax The Spark Max the analog sensor is attached to.
         * @param analogSensor The analog sensor to configure.
         * @param factor The conversion factor which will be multiplied by volts.
         */
        public static void setPositionConversionFactor(CANSparkMax sparkMax, SparkMaxAnalogSensor analogSensor, double factor) {
            set(
                sparkMax,
                () -> analogSensor.setPositionConversionFactor(factor),
                () -> Math2.epsilonEquals(analogSensor.getPositionConversionFactor(), factor, EPSILON),
                "Analog Sensor Position Conversion Factor"
            );
        }

        /**
         * Sets the conversion factor for the velocity of the analog sensor. By default, revolutions per volt
         * second is {@code 1}. Changing the velocity conversion factor will also change the velocity units.
         * @param sparkMax The Spark Max the analog sensor is attached to.
         * @param analogSensor The analog sensor to configure.
         * @param factor The conversion factor which will be multiplied by volts per second.
         */
        public static void setVelocityConversionFactor(CANSparkMax sparkMax, SparkMaxAnalogSensor analogSensor, double factor) {
            set(
                sparkMax,
                () -> analogSensor.setVelocityConversionFactor(factor),
                () -> Math2.epsilonEquals(analogSensor.getVelocityConversionFactor(), factor, EPSILON),
                "Analog Sensor Velocity Conversion Factor"
            );
        }
    }

    /**
     * Spark Max limit switch configuration utilities.
     */
    public static final class LimitSwitch {

        private LimitSwitch() {
            throw new UnsupportedOperationException("This is a utility class!");
        }

        /**
         * Enables or disables controller shutdown based on the limit switch.
         * @param sparkMax The Spark Max the limit switch is attached to.
         * @param limitSwitch The limit switch to configure.
         * @param enable Enable/disable motor shutdown based on the limit switch state.
         */
        public static void enableLimitSwitch(CANSparkMax sparkMax, SparkMaxLimitSwitch limitSwitch, boolean enable) {
            set(
                sparkMax,
                () -> limitSwitch.enableLimitSwitch(enable),
                () -> limitSwitch.isLimitSwitchEnabled() == enable,
                "Enable Limit Switch"
            );
        }
    }

    /**
     * Spark Max PID controller configuration utilities.
     */
    public static final class PIDController {

        private PIDController() {
            throw new UnsupportedOperationException("This is a utility class!");
        }

        /**
         * Sets the derivative gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The derivative gain value, must be positive.
         */
        public static void setD(CANSparkMax sparkMax, double gain) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setD(gain),
                () -> Math2.epsilonEquals(pidController.getD(), gain, EPSILON),
                "PID Controller D Gain"
            );
        }

        /**
         * Sets the derivative gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The derivative gain value, must be positive.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setD(CANSparkMax sparkMax, double gain, int slotId) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setD(gain, slotId),
                () -> Math2.epsilonEquals(pidController.getD(slotId), gain, EPSILON),
                "PID Controller D Gain (Slot " + slotId + ")"
            );
        }

        /**
         * Sets the the derivative filter constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The derivative filter value, must be a positive number between {@code 0} and {@code 1}.
         */
        public static void setDFilter(CANSparkMax sparkMax, double gain) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setDFilter(gain),
                () -> Math2.epsilonEquals(pidController.getDFilter(0), gain, EPSILON),
                "PID Controller D Filter"
            );
        }

        /**
         * Sets the the derivative filter constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The derivative filter value, must be a positive number between {@code 0} and {@code 1}.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setDFilter(CANSparkMax sparkMax, double gain, int slotId) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setDFilter(gain, slotId),
                () -> Math2.epsilonEquals(pidController.getDFilter(slotId), gain, EPSILON),
                "PID Controller D Filter (Slot " + slotId + ")"
            );
        }

        /**
         * Sets the controller's feedback device. The default feedback device in brushless mode
         * is assumed to be the integrated encoder and the default feedback device in brushed
         * mode is assumed to be a quadrature encoder. This is used to changed to another
         * feedback device for the controller, such as an analog sensor. If there is a limited
         * range on the feedback sensor that should be observed by the PIDController, it can be
         * set by calling {@code setFeedbackSensorRange()} on the sensor object.
         * @param sparkMax The Spark Max to configure.
         * @param sensor The sensor to use as a feedback device.
         */
        public static void setFeedbackDevice(CANSparkMax sparkMax, MotorFeedbackSensor sensor) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(sparkMax, () -> pidController.setFeedbackDevice(sensor), "PID Controller Feedback Device");
        }

        /**
         * Sets the  feed-forward gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The  feed-forward gain value, must be positive.
         */
        public static void setFF(CANSparkMax sparkMax, double gain) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setFF(gain),
                () -> Math2.epsilonEquals(pidController.getFF(), gain, EPSILON),
                "PID Controller FF Gain"
            );
        }

        /**
         * Sets the  feed-forward gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The  feed-forward gain value, must be positive.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setFF(CANSparkMax sparkMax, double gain, int slotId) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setFF(gain, slotId),
                () -> Math2.epsilonEquals(pidController.getFF(slotId), gain, EPSILON),
                "PID Controller FF Gain (Slot " + slotId + ")"
            );
        }

        /**
         * Sets the integral gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The integral gain value, must be positive.
         */
        public static void setI(CANSparkMax sparkMax, double gain) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setI(gain),
                () -> Math2.epsilonEquals(pidController.getI(), gain, EPSILON),
                "PID Controller I Gain"
            );
        }

        /**
         * Sets the integral gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The integral gain value, must be positive.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setI(CANSparkMax sparkMax, double gain, int slotId) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setI(gain, slotId),
                () -> Math2.epsilonEquals(pidController.getI(slotId), gain, EPSILON),
                "PID Controller I Gain (Slot " + slotId + ")"
            );
        }

        /**
         * Sets the IZone range of the PIDF controller on the Spark Max. This value specifies the
         * range the error must be within for the integral constant to take effect.
         * @param sparkMax The Spark Max to configure.
         * @param iZone The I zone value, must be positive. Set to {@code 0} to disable.
         */
        public static void setIZone(CANSparkMax sparkMax, double iZone) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setIZone(iZone),
                () -> Math2.epsilonEquals(pidController.getIZone(), iZone, EPSILON),
                "PID Controller I Zone"
            );
        }

        /**
         * Sets the IZone range of the PIDF controller on the Spark Max. This value specifies the
         * range the error must be within for the integral constant to take effect.
         * @param sparkMax The Spark Max to configure.
         * @param iZone The I zone value, must be positive. Set to {@code 0} to disable.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setIZone(CANSparkMax sparkMax, double iZone, int slotId) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setIZone(iZone, slotId),
                () -> Math2.epsilonEquals(pidController.getIZone(slotId), iZone, EPSILON),
                "PID Controller I Zone (Slot " + slotId + ")"
            );
        }

        /**
         * Sets the min amd max output for the closed loop mode.
         * @param sparkMax The Spark Max to configure.
         * @param min Reverse power minimum to allow the controller to output.
         * @param max Forward power maximum to allow the controller to output.
         */
        public static void setOutputRange(CANSparkMax sparkMax, double min, double max) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setOutputRange(min, max),
                () ->
                    Math2.epsilonEquals(pidController.getOutputMin(), min, EPSILON) &&
                    Math2.epsilonEquals(pidController.getOutputMax(), max, EPSILON),
                "PID Controller Output Range"
            );
        }

        /**
         * Sets the min amd max output for the closed loop mode.
         * @param sparkMax The Spark Max to configure.
         * @param min Reverse power minimum to allow the controller to output.
         * @param max Forward power maximum to allow the controller to output.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setOutputRange(CANSparkMax sparkMax, double min, double max, int slotId) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setOutputRange(min, max, slotId),
                () ->
                    Math2.epsilonEquals(pidController.getOutputMin(slotId), min, EPSILON) &&
                    Math2.epsilonEquals(pidController.getOutputMax(slotId), max, EPSILON),
                "PID Controller Output Range (Slot " + slotId + ")"
            );
        }

        /**
         * Sets the proportional gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The proportional gain value, must be positive.
         */
        public static void setP(CANSparkMax sparkMax, double gain) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setP(gain),
                () -> Math2.epsilonEquals(pidController.getP(), gain, EPSILON),
                "PID Controller P Gain"
            );
        }

        /**
         * Sets the proportional gain constant of the PIDF controller on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param gain The proportional gain value, must be positive.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setP(CANSparkMax sparkMax, double gain, int slotId) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setP(gain, slotId),
                () -> Math2.epsilonEquals(pidController.getP(slotId), gain, EPSILON),
                "PID Controller P Gain (Slot " + slotId + ")"
            );
        }

        /**
         * Enables or disables PID Wrapping for position closed loop control.
         * @param sparkMax The Spark Max to configure.
         * @param enable Whether position PID wrapping should be enabled.
         */
        public static void setPositionPIDWrappingEnabled(CANSparkMax sparkMax, boolean enable) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setPositionPIDWrappingEnabled(enable),
                () -> pidController.getPositionPIDWrappingEnabled() == enable,
                "PID Controller Position PID Wrapping Enabled"
            );
        }

        /**
         * Sets the maximum input value for PID Wrapping with position closed loop control.
         * @param sparkMax The Spark Max to configure.
         * @param max The value of max input for the position.
         */
        public static void setPositionPIDWrappingMaxInput(CANSparkMax sparkMax, double max) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setPositionPIDWrappingMaxInput(max),
                () -> Math2.epsilonEquals(pidController.getPositionPIDWrappingMaxInput(), max, EPSILON),
                "PID Controller Position PID Wrapping Max Input"
            );
        }

        /**
         * Sets the minimum input value for PID Wrapping with position closed loop control.
         * @param sparkMax The Spark Max to configure.
         * @param min The value of min input for the position.
         */
        public static void setPositionPIDWrappingMinInput(CANSparkMax sparkMax, double min) {
            SparkMaxPIDController pidController = sparkMax.getPIDController();
            set(
                sparkMax,
                () -> pidController.setPositionPIDWrappingMinInput(min),
                () -> Math2.epsilonEquals(pidController.getPositionPIDWrappingMinInput(), min, EPSILON),
                "PID Controller Position PID Wrapping Min Input"
            );
        }

        /**
         * Sets the minimum and maximum input values for PID Wrapping with position closed loop control.
         * @param sparkMax The Spark Max to configure.
         * @param min The value of min input for the position.
         * @param max The value of max input for the position.
         */
        public static void setPositionPIDWrappingInputLimits(CANSparkMax sparkMax, double min, double max) {
            setPositionPIDWrappingMinInput(sparkMax, min);
            setPositionPIDWrappingMaxInput(sparkMax, max);
        }

        /**
         * Sets PID gains on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param pGain The proportional gain value, must be positive.
         * @param iGain The integral gain value, must be positive.
         * @param dGain The derivative gain value, must be positive.
         */
        public static void setPID(CANSparkMax sparkMax, double pGain, double iGain, double dGain) {
            setP(sparkMax, pGain);
            setI(sparkMax, iGain);
            setD(sparkMax, dGain);
        }

        /**
         * Sets PID gains on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param pGain The proportional gain value, must be positive.
         * @param iGain The integral gain value, must be positive.
         * @param dGain The derivative gain value, must be positive.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setPID(CANSparkMax sparkMax, double pGain, double iGain, double dGain, int slotId) {
            setP(sparkMax, pGain, slotId);
            setI(sparkMax, iGain, slotId);
            setD(sparkMax, dGain, slotId);
        }

        /**
         * Sets PIDF gains on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param pGain The proportional gain value, must be positive.
         * @param iGain The integral gain value, must be positive.
         * @param dGain The derivative gain value, must be positive.
         * @param ffGain The feed-forward gain value, must be positive.
         */
        public static void setPIDF(CANSparkMax sparkMax, double pGain, double iGain, double dGain, double ffGain) {
            setP(sparkMax, pGain);
            setI(sparkMax, iGain);
            setD(sparkMax, dGain);
            setFF(sparkMax, ffGain);
        }

        /**
         * Sets PIDF gains on the Spark Max.
         * @param sparkMax The Spark Max to configure.
         * @param pGain The proportional gain value, must be positive.
         * @param iGain The integral gain value, must be positive.
         * @param dGain The derivative gain value, must be positive.
         * @param ffGain The feed-forward gain value, must be positive.
         * @param slotId The gain schedule slot, the value is a number between {@code 0} and {@code 3}.
         */
        public static void setPIDF(CANSparkMax sparkMax, double pGain, double iGain, double dGain, double ffGain, int slotId) {
            setP(sparkMax, pGain, slotId);
            setI(sparkMax, iGain, slotId);
            setD(sparkMax, dGain, slotId);
            setFF(sparkMax, ffGain, slotId);
        }
    }

    /**
     * Gets the currently configured loop period.
     */
    public static double getPeriod() {
        return period;
    }

    /**
     * Sets the loop period used by the robot.
     * This modifies status frame periods used for Spark Max controllers.
     * This method must be called before setting/enabling frames.
     * @param period The robot's loop period in seconds.
     */
    public static void setPeriod(double period) {
        RevUtil.period = period;
    }

    /**
     * Prints successful configurations to stdout.
     * Useful for debugging, should be ran after initializing all hardware.
     */
    public static void printSuccess() {
        System.out.println("\nSuccessfully configured " + setSuccess.size() + " options on Spark Max controllers:");
        for (String successString : setSuccess) {
            System.out.println("\t" + successString);
        }
        System.out.println("\n");
        setSuccess.clear();
    }

    /**
     * Wrapper for setting config values.
     * @param setter The config setter.
     * @param name The name of the config value.
     */
    private static void set(CANSparkMax sparkMax, Supplier<REVLibError> setter, String name) {
        set(sparkMax, setter, () -> true, false, name);
    }

    /**
     * Wrapper for setting config values.
     * If the checker ever returns {@code true}, subsequent config
     * calls are assumed to be unnecessary and will be skipped.
     * @param setter The config setter.
     * @param checker A boolean supplier that returns {@code true} if the value was set.
     * @param name The name of the config value.
     */
    private static void set(CANSparkMax sparkMax, Supplier<REVLibError> setter, Supplier<Boolean> checker, String name) {
        set(sparkMax, setter, checker, true, name);
    }

    /**
     * Wrapper for setting config values.
     * @param setter The config setter.
     * @param checker A boolean supplier that returns {@code true} if the value was (possibly) set.
     * @param trustCheck If the checker can be trusted and redundant calls to the controller are unnecessary.
     * @param name The name of the config value.
     */
    private static void set(
        CANSparkMax sparkMax,
        Supplier<REVLibError> setter,
        Supplier<Boolean> checker,
        boolean trustCheck,
        String name
    ) {
        set(sparkMax, setter, checker, trustCheck, name, new String[SET_ITERATIONS], SET_ITERATIONS);
    }

    /**
     * Wrapper for setting config values.
     * @param setter The config setter.
     * @param checker A boolean supplier that returns {@code true} if the value was (possibly) set.
     * @param trustCheck If the checker can be trusted and redundant calls to the controller are unnecessary.
     * @param name The name of the config value.
     * @param results A mutable array to push results to.
     * @param iterationsLeft The number of iterations left before failing.
     */
    private static void set(
        CANSparkMax sparkMax,
        Supplier<REVLibError> setter,
        Supplier<Boolean> checker,
        boolean trustCheck,
        String name,
        String[] results,
        int iterationsLeft
    ) {
        try {
            REVLibError status = setter.get();
            if (!RobotBase.isSimulation()) {
                try {
                    Thread.sleep((long) CHECK_SLEEP);
                } catch (Exception e) {}
            }

            boolean check = checker.get();
            results[results.length - iterationsLeft] = status.name() + (!check ? " (Failed Check)" : "");

            if (trustCheck && check && REVLibError.kOk.equals(status)) {
                for (int i = 0; i < results.length; i++) {
                    if (i <= results.length - iterationsLeft) continue;
                    results[i] = "Skipped";
                }
                iterationsLeft = 1;
            }
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), true);
            results[results.length - iterationsLeft] = e.getClass().getSimpleName();
        }

        iterationsLeft--;
        if (iterationsLeft <= 0) {
            String resultsString = "";
            boolean hadFailure = false;
            for (int i = 0; i < results.length; i++) {
                if (
                    !(results[i].equals(REVLibError.kOk.name()) || results[i].equals("Skipped")) &&
                    !(RobotBase.isSimulation() && results[i].startsWith(REVLibError.kParamMismatchType.name()))
                ) hadFailure = true;
                resultsString += results[i];
                if (i != results.length - 1) resultsString += ", ";
            }

            if (hadFailure) {
                DriverStation.reportWarning(
                    "Failure(s) encountered while configuring \"" +
                    name +
                    "\" on Spark Max with Device ID " +
                    sparkMax.getDeviceId() +
                    ": " +
                    resultsString,
                    true
                );
            } else {
                setSuccess.add("ID " + sparkMax.getDeviceId() + " \"" + name + "\": " + resultsString);
            }
        } else {
            set(sparkMax, setter, checker, trustCheck, name, results, iterationsLeft);
        }
    }

    private static final class SuccessComparator implements Comparator<String> {

        private static SuccessComparator instance;
        private final Collator localeComparator = Collator.getInstance();

        public static SuccessComparator getInstance() {
            if (instance == null) instance = new SuccessComparator();
            return instance;
        }

        private SuccessComparator() {}

        @Override
        public int compare(String arg0, String arg1) {
            int idDiff = Integer.parseInt(arg0.split(" ")[1]) - Integer.parseInt(arg1.split(" ")[1]);
            if (idDiff == 0) {
                int localeDiff = localeComparator.compare(arg0, arg1);
                if (localeDiff == 0) return 1; else return localeDiff;
            } else {
                return idDiff;
            }
        }
    }
}
