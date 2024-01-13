package org.team340.lib;

import com.revrobotics.CANSparkBase.FaultID;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.team340.lib.util.Math2;

// TODO Faults

/**
 * {@link Sendable} wrappers for hardware.
 * Also provides methods for tracking faults and power usage.
 */
final class HardwareSendables {

    private HardwareSendables() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Generic hardware.
     */
    public abstract static class Hardware implements Sendable {

        private final String key;
        private final String label;
        private final String api;

        /**
         * @param key The key to use in network tables. It is recommended to have this key be related to the bus and device ID the hardware is accessed through. For example, {@code "CAN-10"}.
         * @param label The label to use. Shown in the dashboard.
         * @param api The API used for interfacing with the hardware in code.
         */
        public Hardware(String key, String label, Object api) {
            this.key = key;
            this.label = label;
            this.api = api.getClass().getSimpleName();
        }

        /**
         * Adds the hardware to the dashboard.
         * @param subsystem The subsystem the hardware is associated with.
         */
        public void addToDashboard(GRRSubsystem subsystem) {
            GRRDashboard.addHardware(subsystem, this);
        }

        /**
         * Returns the hardware's key.
         */
        public String getKey() {
            return key;
        }

        /**
         * Gets hardware faults.
         */
        public String[] getFaults() {
            return new String[0];
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.publishConstString(".label", label);
            builder.publishConstString(".api", api);
            builder.addStringArrayProperty("faults", this::getFaults, null);
        }
    }

    /**
     * Generic hardware that has trackable power consumption.
     */
    public abstract static class PoweredHardware extends Hardware {

        private final Supplier<Double> power;
        private final Supplier<Double> voltage;
        private final Supplier<Double> current;
        private final ReentrantLock usageMutex = new ReentrantLock();
        private final Timer usageTimer = new Timer();
        private double usage = 0.0;
        private double lastPower = 0.0;
        private boolean updatedPower = false;

        /**
         * @param key The key to use in network tables. It is recommended to have this key be related to the bus and device ID the hardware is accessed through. For example, {@code "CAN-10"}.
         * @param label The label to use. Shown in the dashboard.
         * @param api The API used for interfacing with the hardware in code.
         * @param voltage A supplier for the device's voltage.
         * @param current A supplier for the device's current in amps.
         */
        public PoweredHardware(String key, String label, Object api, Supplier<Double> voltage, Supplier<Double> current) {
            this(key, label, api, () -> voltage.get() * current.get(), voltage, current);
        }

        /**
         * @param key The key to use in network tables. It is recommended to have this key be related to the bus and device ID the hardware is accessed through. For example, {@code "CAN-10"}.
         * @param label The label to use. Shown in the dashboard.
         * @param api The API used for interfacing with the hardware in code.
         * @param power A supplier for the device's power in watts.
         * @param voltage A supplier for the device's voltage.
         * @param current A supplier for the device's current in amps.
         */
        public PoweredHardware(
            String key,
            String label,
            Object api,
            Supplier<Double> power,
            Supplier<Double> voltage,
            Supplier<Double> current
        ) {
            super(key, label, api);
            this.power = power;
            this.voltage = voltage;
            this.current = current;
        }

        /**
         * Adds the hardware to the dashboard.
         * @param subsystem The subsystem the hardware is associated with.
         */
        public void addToDashboard(GRRSubsystem subsystem) {
            GRRDashboard.addHardware(subsystem, this);
        }

        /**
         * Updates the hardware's power usage counter.
         * Call this periodically to ensure more accurate tracking. Smaller intervals results in better accuracy.
         */
        public void updatePowerUsage() {
            try {
                usageMutex.lock();
                lastPower = power.get();
                usage += (usageTimer.get() / 3600.0) * lastPower;
                usageTimer.restart();
                updatedPower = true;
            } finally {
                usageMutex.unlock();
            }
        }

        /**
         * Gets the hardware's power usage since startup in watt hours.
         */
        public double getPowerUsage() {
            return usage;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addDoubleProperty(
                "power",
                () -> {
                    double p;
                    try {
                        usageMutex.lock();
                        if (updatedPower) {
                            updatedPower = false;
                            p = lastPower;
                        } else {
                            p = power.get();
                        }
                    } finally {
                        usageMutex.unlock();
                    }

                    return Math2.toFixed(p);
                },
                null
            );
            builder.addDoubleProperty("powerUsage", () -> Math2.toFixed(usage), null);
            builder.addDoubleProperty("voltage", () -> Math2.toFixed(voltage.get()), null);
            builder.addDoubleProperty("current", () -> Math2.toFixed(current.get()), null);
        }
    }

    /**
     * A generic motor.
     */
    public abstract static class Motor extends PoweredHardware {

        private final Supplier<Double> output;
        private final Supplier<Double> temperature;
        private final Supplier<Double> velocity;
        private final Supplier<Double> position;

        /**
         * @param key The key to use in network tables. It is recommended to have this key be related to the bus and device ID the hardware is accessed through. For example, {@code "CAN-10"}.
         * @param label The label to use. Shown in the dashboard.
         * @param api The API used for interfacing with the hardware in code.
         * @param voltage A supplier for the device's voltage.
         * @param current A supplier for the device's current.
         * @param output A supplier for the motor's applied output. Should be a value from {@code -1.0} to {@code 1.0}.
         * @param temperature A supplier for the temperature in celsius of the motor or the motor's controller (whichever is accessible, preferably the motor).
         * @param velocity A supplier for the motor's velocity.
         * @param position A supplier for the motor's position.
         */
        public Motor(
            String key,
            String label,
            Object api,
            Supplier<Double> voltage,
            Supplier<Double> current,
            Supplier<Double> output,
            Supplier<Double> temperature,
            Supplier<Double> velocity,
            Supplier<Double> position
        ) {
            super(key, label, api, voltage, current);
            this.output = output;
            this.temperature = temperature;
            this.velocity = velocity;
            this.position = position;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addDoubleProperty("output", () -> Math2.toFixed(output.get()), null);
            builder.addDoubleProperty("temperature", () -> Math2.toFixed(temperature.get()), null);
            builder.addDoubleProperty("velocity", () -> Math2.toFixed(velocity.get()), null);
            builder.addDoubleProperty("position", () -> Math2.toFixed(position.get()), null);
        }
    }

    /**
     * A generic encoder.
     */
    public abstract static class Encoder extends Hardware {

        private final Supplier<Double> velocity;
        private final Supplier<Double> position;

        /**
         * @param key The key to use in network tables. It is recommended to have this key be related to the bus and device ID the hardware is accessed through. For example, {@code "CAN-10"}.
         * @param label The label to use. Shown in the dashboard.
         * @param api The API used for interfacing with the hardware in code.
         * @param velocity A supplier for the encoder's velocity.
         * @param position A supplier for the encoder's position. If the encoder is absolute, prefer the absolute position.
         */
        public Encoder(String key, String label, Object api, Supplier<Double> velocity, Supplier<Double> position) {
            super(key, label, api);
            this.velocity = velocity;
            this.position = position;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addDoubleProperty("velocity", () -> Math2.toFixed(velocity.get()), null);
            builder.addDoubleProperty("position", () -> Math2.toFixed(position.get()), null);
        }
    }

    /**
     * A generic IMU.
     */
    public abstract static class IMU extends Hardware {

        private final Supplier<Double> yaw;
        private final Supplier<Double> pitch;
        private final Supplier<Double> roll;

        /**
         * @param key The key to use in network tables. It is recommended to have this key be related to the bus and device ID the hardware is accessed through. For example, {@code "CAN-10"}.
         * @param label The label to use. Shown in the dashboard.
         * @param api The API used for interfacing with the hardware in code.
         * @param yaw A supplier for the IMU's yaw in radians.
         * @param pitch A supplier for the IMU's pitch in radians.
         * @param roll A supplier for the IMU's roll in radians.
         */
        public IMU(String key, String label, Object api, Supplier<Double> yaw, Supplier<Double> pitch, Supplier<Double> roll) {
            super(key, label, api);
            this.yaw = yaw;
            this.pitch = pitch;
            this.roll = roll;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addDoubleProperty("yaw", () -> Math2.toFixed(yaw.get()), null);
            builder.addDoubleProperty("pitch", () -> Math2.toFixed(pitch.get()), null);
            builder.addDoubleProperty("roll", () -> Math2.toFixed(roll.get()), null);
        }
    }

    /**
     * A Spark Max {@link Sendable}.
     */
    public static final class SparkMax extends Motor {

        private final CANSparkMax sparkMax;

        /**
         * Create the Spark Max sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param sparkMax The Spark Max.
         */
        public SparkMax(String label, CANSparkMax sparkMax) {
            this(label, sparkMax, sparkMax.getEncoder());
        }

        private SparkMax(String label, CANSparkMax sparkMax, RelativeEncoder relativeEncoder) {
            super(
                "CAN-" + sparkMax.getDeviceId(),
                label,
                sparkMax,
                () -> sparkMax.getBusVoltage(),
                () -> sparkMax.getOutputCurrent(),
                () -> sparkMax.getAppliedOutput(),
                () -> sparkMax.getMotorTemperature(),
                () -> relativeEncoder.getVelocity(),
                () -> relativeEncoder.getPosition()
            );
            this.sparkMax = sparkMax;
        }

        @Override
        public String[] getFaults() {
            int faults = sparkMax.getStickyFaults() | sparkMax.getFaults();
            List<String> faultStrings = new ArrayList<>();
            for (int i = 0; i < 16; i++) {
                if (((faults >> i) & 1) == 1) {
                    faultStrings.add(FaultID.fromId(i).name());
                }
            }

            return faultStrings.toArray(new String[faultStrings.size()]);
        }
    }

    /**
     * A Spark Flex {@link Sendable}.
     */
    public static final class SparkFlex extends Motor {

        private final CANSparkFlex sparkFlex;

        /**
         * Create the Spark Flex sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param sparkFlex The Spark Flex.
         */
        public SparkFlex(String label, CANSparkFlex sparkFlex) {
            this(label, sparkFlex, sparkFlex.getEncoder());
        }

        private SparkFlex(String label, CANSparkFlex sparkFlex, RelativeEncoder relativeEncoder) {
            super(
                "CAN-" + sparkFlex.getDeviceId(),
                label,
                sparkFlex,
                () -> sparkFlex.getBusVoltage(),
                () -> sparkFlex.getOutputCurrent(),
                () -> sparkFlex.getAppliedOutput(),
                () -> sparkFlex.getMotorTemperature(),
                () -> relativeEncoder.getVelocity(),
                () -> relativeEncoder.getPosition()
            );
            this.sparkFlex = sparkFlex;
        }

        @Override
        public String[] getFaults() {
            int faults = sparkFlex.getStickyFaults() | sparkFlex.getFaults();
            List<String> faultStrings = new ArrayList<>();
            for (int i = 0; i < 16; i++) {
                if (((faults >> i) & 1) == 1) {
                    faultStrings.add(FaultID.fromId(i).name());
                }
            }

            return faultStrings.toArray(new String[faultStrings.size()]);
        }
    }

    /**
     * A Talon SRX {@link Sendable}.
     */
    public static final class TalonSRX extends Motor {

        /**
         * Create the Talon SRX sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param talonSRX The Talon SRX.
         */
        public TalonSRX(String label, com.ctre.phoenix.motorcontrol.can.TalonSRX talonSRX) {
            super(
                "CAN-" + talonSRX.getDeviceID(),
                label,
                talonSRX,
                () -> talonSRX.getBusVoltage(),
                () -> talonSRX.getStatorCurrent(),
                () -> talonSRX.getMotorOutputPercent() / 100.0,
                () -> talonSRX.getTemperature(),
                () -> talonSRX.getSelectedSensorVelocity(),
                () -> talonSRX.getSelectedSensorPosition()
            );
        }
    }

    /**
     * A Talon FX {@link Sendable}.
     */
    public static final class TalonFX extends Motor {

        /**
         * Create the Talon FX sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param talonFX The Talon FX.
         */
        public TalonFX(String label, com.ctre.phoenix6.hardware.TalonFX talonFX) {
            super(
                "CAN-" + talonFX.getDeviceID(),
                label,
                talonFX,
                () -> talonFX.getSupplyVoltage().getValue(),
                () -> talonFX.getSupplyCurrent().getValue(),
                () -> talonFX.getDutyCycle().getValue(),
                () -> talonFX.getDeviceTemp().getValue(),
                () -> talonFX.getRotorVelocity().getValue(),
                () -> talonFX.getRotorPosition().getValue()
            );
        }
    }

    /**
     * A Spark Absolute Encoder {@link Sendable}.
     */
    public static final class SparkAbsoluteEncoder extends Encoder {

        /**
         * Create the Spark Max Absolute Encoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param sparkMax The Spark Max the encoder is attached to.
         * @param absoluteEncoder The absolute encoder.
         */
        public SparkAbsoluteEncoder(String label, CANSparkMax sparkMax, com.revrobotics.SparkAbsoluteEncoder absoluteEncoder) {
            this(label, sparkMax.getDeviceId(), absoluteEncoder);
        }

        /**
         * Create the Spark Flex Absolute Encoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param sparkFlex The Spark Flex the encoder is attached to.
         * @param absoluteEncoder The absolute encoder.
         */
        public SparkAbsoluteEncoder(String label, CANSparkFlex sparkFlex, com.revrobotics.SparkAbsoluteEncoder absoluteEncoder) {
            this(label, sparkFlex.getDeviceId(), absoluteEncoder);
        }

        /**
         * Create the Spark Absolute Encoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param sparkDeviceId The device ID of the Spark the encoder is attached to.
         * @param absoluteEncoder The absolute encoder.
         */
        public SparkAbsoluteEncoder(String label, int sparkDeviceId, com.revrobotics.SparkAbsoluteEncoder absoluteEncoder) {
            super(
                "CAN-" + sparkDeviceId + "-AbsoluteEncoder",
                label,
                absoluteEncoder,
                () -> absoluteEncoder.getVelocity(),
                () -> absoluteEncoder.getPosition()
            );
        }
    }

    /**
     * A CANcoder {@link Sendable}.
     */
    public static final class CANcoder extends Encoder {

        /**
         * Create the CANcoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param canCoder The CANcoder.
         */
        public CANcoder(String label, com.ctre.phoenix6.hardware.CANcoder canCoder) {
            super(
                "CAN-" + canCoder.getDeviceID(),
                label,
                canCoder,
                () -> canCoder.getVelocity().getValue(),
                () -> canCoder.getAbsolutePosition().getValue()
            );
        }
    }

    /**
     * A DIO attached Encoder {@link Sendable}.
     */
    public static final class DIOEncoder extends Encoder {

        /**
         * Create the DIO attached encoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param sourceA The encoder's A channel source.
         * @param sourceB The encoder's B channel source.
         * @param encoder The encoder.
         */
        public DIOEncoder(String label, DigitalSource sourceA, DigitalSource sourceB, edu.wpi.first.wpilibj.Encoder encoder) {
            this(label, sourceA.getChannel(), sourceB.getChannel(), encoder);
        }

        /**
         * Create the DIO attached encoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param sourceA The encoder's A channel source.
         * @param sourceB The encoder's B channel source.
         * @param indexSource The encoder's index channel source.
         * @param encoder The encoder.
         */
        public DIOEncoder(
            String label,
            DigitalSource sourceA,
            DigitalSource sourceB,
            DigitalSource indexSource,
            edu.wpi.first.wpilibj.Encoder encoder
        ) {
            this(label, sourceA.getChannel(), sourceB.getChannel(), indexSource.getChannel(), encoder);
        }

        /**
         * Create the DIO attached encoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param channelA The encoder's DIO A channel.
         * @param channelB The encoder's DIO B channel.
         * @param encoder The encoder.
         */
        public DIOEncoder(String label, int channelA, int channelB, edu.wpi.first.wpilibj.Encoder encoder) {
            super("DIO-" + channelA + "-" + channelB, label, encoder, () -> encoder.getRate(), () -> encoder.getDistance());
        }

        /**
         * Create the DIO attached encoder sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param channelA The encoder's DIO A channel.
         * @param channelB The encoder's DIO B channel.
         * @param indexChannel The encoder's DIO index channel.
         * @param encoder The encoder.
         */
        public DIOEncoder(String label, int channelA, int channelB, int indexChannel, edu.wpi.first.wpilibj.Encoder encoder) {
            super(
                "DIO-" + channelA + "-" + channelB + "-" + indexChannel,
                label,
                encoder,
                () -> encoder.getRate(),
                () -> encoder.getDistance()
            );
        }
    }

    /**
     * A ADIS16470 {@link Sendable}.
     */
    public static final class ADIS16470 extends IMU {

        /**
         * Create the ADIS16470 sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param adis16470 The ADIS16470.
         */
        public ADIS16470(String label, edu.wpi.first.wpilibj.ADIS16470_IMU adis16470) {
            super(
                "SPI-" + adis16470.getPort(),
                label,
                adis16470,
                () -> Math.toRadians(adis16470.getAngle(adis16470.getYawAxis())),
                () -> Math.toRadians(adis16470.getAngle(adis16470.getPitchAxis())),
                () -> Math.toRadians(adis16470.getAngle(adis16470.getRollAxis()))
            );
        }
    }

    /**
     * A Pigeon 2 {@link Sendable}.
     */
    public static final class Pigeon2 extends IMU {

        /**
         * Create the Pigeon 2 sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param pigeon2 The Pigeon 2.
         */
        public Pigeon2(String label, com.ctre.phoenix6.hardware.Pigeon2 pigeon2) {
            super(
                "CAN-" + pigeon2.getDeviceID(),
                label,
                pigeon2,
                () -> Math.toRadians(pigeon2.getYaw().getValue()),
                () -> Math.toRadians(pigeon2.getPitch().getValue()),
                () -> Math.toRadians(pigeon2.getRoll().getValue())
            );
        }
    }

    /**
     * A Digital Input {@link Sendable}.
     */
    public static final class DigitalInput extends Hardware {

        private final Supplier<Boolean> value;

        /**
         * Create the Digital Input sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param digitalInput The Digital Input.
         */
        public DigitalInput(String label, edu.wpi.first.wpilibj.DigitalInput digitalInput) {
            super("DIO-" + digitalInput.getChannel(), label, digitalInput);
            value = digitalInput::get;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addBooleanProperty("value", value::get, null);
        }
    }

    /**
     * A Pneumatic Hub {@link Sendable}.
     */
    public static final class PneumaticHub extends PoweredHardware {

        private final Supplier<Double> pressure;
        private final Supplier<Boolean> pressureSwitchOn;
        private final Supplier<Boolean> compressorOn;

        /**
         * Create the Pneumatic Hub sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param pneumaticHub The Pneumatic Hub.
         */
        public PneumaticHub(String label, edu.wpi.first.wpilibj.PneumaticHub pneumaticHub) {
            super(
                "PneumaticController-" + pneumaticHub.getModuleNumber(),
                label,
                pneumaticHub,
                () ->
                    (pneumaticHub.getInputVoltage() * pneumaticHub.getCompressorCurrent()) +
                    (pneumaticHub.getSolenoidsVoltage() * pneumaticHub.getSolenoidsTotalCurrent()),
                () -> pneumaticHub.getInputVoltage(),
                () ->
                    pneumaticHub.getCompressorCurrent() +
                    ((pneumaticHub.getSolenoidsVoltage() / pneumaticHub.getInputVoltage()) * pneumaticHub.getSolenoidsTotalCurrent())
            );
            pressure = () -> pneumaticHub.getPressure(0);
            pressureSwitchOn = pneumaticHub::getPressureSwitch;
            compressorOn = pneumaticHub::getCompressor;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addDoubleProperty("pressure", () -> Math2.toFixed(pressure.get()), null);
            builder.addBooleanProperty("pressureSwitchOn", pressureSwitchOn::get, null);
            builder.addBooleanProperty("compressorOn", compressorOn::get, null);
        }
    }

    /**
     * A Solenoid {@link Sendable}.
     */
    public static final class Solenoid extends Hardware {

        private final Supplier<Boolean> value;

        /**
         * Create the Solenoid sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param solenoid The Solenoid.
         */
        public Solenoid(String label, edu.wpi.first.wpilibj.Solenoid solenoid) {
            super("Solenoid-" + solenoid.getChannel(), label, solenoid);
            value = solenoid::get;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addBooleanProperty("value", value::get, null);
        }
    }

    /**
     * A Double Solenoid {@link Sendable}.
     */
    public static final class DoubleSolenoid extends Hardware {

        private final Supplier<Integer> value;

        /**
         * Create the Double Solenoid sendable.
         * @param label The label to use. Shown in the dashboard.
         * @param doubleSolenoid The Double Solenoid.
         */
        public DoubleSolenoid(String label, edu.wpi.first.wpilibj.DoubleSolenoid doubleSolenoid) {
            super("Solenoid-" + doubleSolenoid.getFwdChannel() + "-" + doubleSolenoid.getRevChannel(), label, doubleSolenoid);
            value = () -> doubleSolenoid.get().ordinal();
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            super.initSendable(builder);
            builder.addIntegerProperty("value", value::get, null);
        }
    }
}
