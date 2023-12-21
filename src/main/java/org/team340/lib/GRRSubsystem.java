package org.team340.lib;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team340.lib.commands.CommandBuilder;
import org.team340.lib.drivers.imu.ADIS16470;
import org.team340.lib.drivers.imu.ADIS16470.CalibrationTime;
import org.team340.lib.drivers.imu.ADIS16470.IMUAxis;

/**
 * An extension to WPILib's subsystem.
 * Adds factories for creating hardware and automatically adds them to the dashboard.
 */
public abstract class GRRSubsystem extends SubsystemBase {

    protected final String label;

    /**
     * Create the subsystem.
     * @param label The label to give the subsystem. Shown in the dashboard.
     */
    public GRRSubsystem(String label) {
        super();
        this.label = label;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.publishConstString(".label", label);
        builder.publishConstString(".api", "GRRSubsystem");
        builder.addStringProperty("command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "{None}", null);
        builder.addStringProperty("default", () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "{None}", null);
        builder.addBooleanProperty("hasCommand", () -> getCurrentCommand() != null, null);
        builder.addBooleanProperty("hasDefault", () -> getDefaultCommand() != null, null);
    }

    /**
     * Adds the subsystem to the dashboard.
     */
    public void addToDashboard() {
        GRRDashboard.addSubsystem(this);
    }

    /**
     * Creates a command builder that requires this subsystem.
     */
    protected CommandBuilder commandBuilder() {
        return new CommandBuilder(this);
    }

    /**
     * Creates a command builder that requires this subsystem.
     * @param name The name of the command.
     */
    protected CommandBuilder commandBuilder(String name) {
        return new CommandBuilder(name, this);
    }

    /**
     * A command that reports a warning to the driver station.
     * @param warning The warning to report.
     * @param printTrace If {@code true}, appends a stack trace to warning string.
     */
    protected Command reportWarning(String warning, boolean printTrace) {
        return runOnce(() -> DriverStation.reportWarning(warning, printTrace));
    }

    /**
     * A command that reports an error to the driver station.
     * @param error The error to report.
     * @param printTrace If {@code true}, appends a stack trace to error string.
     */
    protected Command reportError(String error, boolean printTrace) {
        return runOnce(() -> DriverStation.reportError(error, printTrace));
    }

    /**
     * Creates a Spark Max.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the Spark Max on the CAN bus.
     * @param type The motor type connected to the controller.
     */
    protected CANSparkMax createSparkMax(String label, int deviceId, MotorType type) {
        CANSparkMax sparkMax = new CANSparkMax(deviceId, type);
        new HardwareSendables.SparkMax(label, sparkMax).addToDashboard(this);
        return sparkMax;
    }

    /**
     * Creates a Talon SRX.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the Talon FX on the CAN bus.
     */
    protected TalonSRX createTalonSRX(String label, int deviceId) {
        TalonSRX talonSRX = new TalonSRX(deviceId);
        new HardwareSendables.TalonSRX(label, talonSRX).addToDashboard(this);
        return talonSRX;
    }

    /**
     * Creates a Talon FX.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the Talon FX on the CAN bus.
     */
    protected TalonFX createTalonFX(String label, int deviceId) {
        TalonFX talonFX = new TalonFX(deviceId);
        new HardwareSendables.TalonFX(label, talonFX).addToDashboard(this);
        return talonFX;
    }

    /**
     * Creates a Talon FX.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the Talon FX on the CAN bus.
     * @param canBus Name of the CAN bus the Talon FX is on.
     */
    protected TalonFX createTalonFX(String label, int deviceId, String canBus) {
        TalonFX talonFX = new TalonFX(deviceId, canBus);
        new HardwareSendables.TalonFX(label, talonFX).addToDashboard(this);
        return talonFX;
    }

    /**
     * Creates a Spark Max attached Absolute Encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param sparkMax The Spark Max the absolute encoder is attached to.
     * @param type The encoder type.
     */
    protected SparkMaxAbsoluteEncoder createSparkMaxAbsoluteEncoder(String label, CANSparkMax sparkMax, SparkMaxAbsoluteEncoder.Type type) {
        SparkMaxAbsoluteEncoder absoluteEncoder = sparkMax.getAbsoluteEncoder(type);
        new HardwareSendables.SparkMaxAbsoluteEncoder(label, sparkMax, absoluteEncoder).addToDashboard(this);
        return absoluteEncoder;
    }

    /**
     * Creates a CANcoder.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the CANcoder on the CAN bus.
     */
    protected CANcoder createCANcoder(String label, int deviceId) {
        CANcoder canCoder = new CANcoder(deviceId);
        new HardwareSendables.CANcoder(label, canCoder).addToDashboard(this);
        return canCoder;
    }

    /**
     * Creates a CANcoder.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the CANcoder on the CAN bus.
     * @param canBus Name of the CAN bus the CANcoder is on.
     */
    protected CANcoder createCANcoder(String label, int deviceId, String canBus) {
        CANcoder canCoder = new CANcoder(deviceId, canBus);
        new HardwareSendables.CANcoder(label, canCoder).addToDashboard(this);
        return canCoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param channelA The encoder's DIO A channel.
     * @param channelB The encoder's DIO B channel.
     */
    protected Encoder createDIOEncoder(String label, int channelA, int channelB) {
        Encoder encoder = new Encoder(channelA, channelB);
        new HardwareSendables.DIOEncoder(label, channelA, channelB, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param channelA The encoder's DIO A channel.
     * @param channelB The encoder's DIO B channel.
     * @param reverseDirection The orientation of the encoder and inverts the output values if necessary so forward represents positive values.
     */
    protected Encoder createDIOEncoder(String label, int channelA, int channelB, boolean reverseDirection) {
        Encoder encoder = new Encoder(channelA, channelB, reverseDirection);
        new HardwareSendables.DIOEncoder(label, channelA, channelB, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param channelA The encoder's DIO A channel.
     * @param channelB The encoder's DIO B channel.
     * @param reverseDirection The orientation of the encoder and inverts the output values if necessary so forward represents positive values.
     * @param encodingType Either k1X, k2X, or k4X to indicate 1X, 2X or 4X decoding. If 4X is selected, then an encoder FPGA object is used and the returned counts will be 4x the encoder spec'd value since all rising and falling edges are counted. If 1X or 2X are selected, then a counter object will be used and the returned value will either exactly match the spec'd count or be double (2x) the spec'd count.
     */
    protected Encoder createDIOEncoder(String label, int channelA, int channelB, boolean reverseDirection, EncodingType encodingType) {
        Encoder encoder = new Encoder(channelA, channelB, reverseDirection, encodingType);
        new HardwareSendables.DIOEncoder(label, channelA, channelB, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param channelA The encoder's DIO A channel.
     * @param channelB The encoder's DIO B channel.
     * @param indexChannel The encoder's DIO index channel.
     */
    protected Encoder createDIOEncoder(String label, int channelA, int channelB, int indexChannel) {
        Encoder encoder = new Encoder(channelA, channelB, indexChannel);
        new HardwareSendables.DIOEncoder(label, channelA, channelB, indexChannel, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param channelA The encoder's DIO A channel.
     * @param channelB The encoder's DIO B channel.
     * @param indexChannel The encoder's DIO index channel.
     * @param reverseDirection The orientation of the encoder and inverts the output values if necessary so forward represents positive values.
     */
    protected Encoder createDIOEncoder(String label, int channelA, int channelB, int indexChannel, boolean reverseDirection) {
        Encoder encoder = new Encoder(channelA, channelB, indexChannel, reverseDirection);
        new HardwareSendables.DIOEncoder(label, channelA, channelB, indexChannel, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param sourceA The encoder's A channel source.
     * @param sourceB The encoder's B channel source.
     */
    protected Encoder createDIOEncoder(String label, DigitalSource sourceA, DigitalSource sourceB) {
        Encoder encoder = new Encoder(sourceA, sourceB);
        new HardwareSendables.DIOEncoder(label, sourceA, sourceB, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param sourceA The encoder's A channel source.
     * @param sourceB The encoder's B channel source.
     * @param reverseDirection The orientation of the encoder and inverts the output values if necessary so forward represents positive values.
     */
    protected Encoder createDIOEncoder(String label, DigitalSource sourceA, DigitalSource sourceB, boolean reverseDirection) {
        Encoder encoder = new Encoder(sourceA, sourceB, reverseDirection);
        new HardwareSendables.DIOEncoder(label, sourceA, sourceB, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param sourceA The encoder's A channel source.
     * @param sourceB The encoder's B channel source.
     * @param reverseDirection The orientation of the encoder and inverts the output values if necessary so forward represents positive values.
     * @param encodingType Either k1X, k2X, or k4X to indicate 1X, 2X or 4X decoding. If 4X is selected, then an encoder FPGA object is used and the returned counts will be 4x the encoder spec'd value since all rising and falling edges are counted. If 1X or 2X are selected, then a counter object will be used and the returned value will either exactly match the spec'd count or be double (2x) the spec'd count.
     */
    protected Encoder createDIOEncoder(
        String label,
        DigitalSource sourceA,
        DigitalSource sourceB,
        boolean reverseDirection,
        EncodingType encodingType
    ) {
        Encoder encoder = new Encoder(sourceA, sourceB, reverseDirection, encodingType);
        new HardwareSendables.DIOEncoder(label, sourceA, sourceB, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param sourceA The encoder's A channel source.
     * @param sourceB The encoder's B channel source.
     * @param indexSource The encoder's index channel source.
     */
    protected Encoder createDIOEncoder(String label, DigitalSource sourceA, DigitalSource sourceB, DigitalSource indexSource) {
        Encoder encoder = new Encoder(sourceA, sourceB, indexSource);
        new HardwareSendables.DIOEncoder(label, sourceA, sourceB, indexSource, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates a DIO encoder.
     * @param label The label to use. Shown in the dashboard.
     * @param sourceA The encoder's A channel source.
     * @param sourceB The encoder's B channel source.
     * @param indexSource The encoder's index channel source.
     * @param reverseDirection The orientation of the encoder and inverts the output values if necessary so forward represents positive values.
     */
    protected Encoder createDIOEncoder(
        String label,
        DigitalSource sourceA,
        DigitalSource sourceB,
        DigitalSource indexSource,
        boolean reverseDirection
    ) {
        Encoder encoder = new Encoder(sourceA, sourceB, indexSource, reverseDirection);
        new HardwareSendables.DIOEncoder(label, sourceA, sourceB, indexSource, encoder).addToDashboard(this);
        return encoder;
    }

    /**
     * Creates an ADIS16470.
     * @param label The label to use. Shown in the dashboard.
     * @param yawAxis The axis that measures the yaw.
     * @param pitchAxis The axis that measures the pitch.
     * @param rollAxis The axis that measures the roll.
     */
    protected ADIS16470 createADIS16470(String label, IMUAxis yawAxis, IMUAxis pitchAxis, IMUAxis rollAxis) {
        ADIS16470 adis16470 = new ADIS16470(yawAxis, pitchAxis, rollAxis);
        new HardwareSendables.ADIS16470(label, adis16470).addToDashboard(this);
        return adis16470;
    }

    /**
     * Creates an ADIS16470.
     * @param label The label to use. Shown in the dashboard.
     * @param yawAxis The axis that measures the yaw.
     * @param pitchAxis The axis that measures the pitch.
     * @param rollAxis The axis that measures the roll.
     * @param port The SPI Port the gyro is plugged into.
     * @param calibrationTime Calibration time.
     */
    protected ADIS16470 createADIS16470(
        String label,
        IMUAxis yawAxis,
        IMUAxis pitchAxis,
        IMUAxis rollAxis,
        SPI.Port port,
        CalibrationTime calibrationTime
    ) {
        ADIS16470 adis16470 = new ADIS16470(yawAxis, pitchAxis, rollAxis, port, calibrationTime);
        new HardwareSendables.ADIS16470(label, adis16470).addToDashboard(this);
        return adis16470;
    }

    /**
     * Creates a Pigeon 2.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the Pigeon 2 on the CAN bus.
     */
    protected Pigeon2 createPigeon2(String label, int deviceId) {
        Pigeon2 pigeon2 = new Pigeon2(deviceId);
        new HardwareSendables.Pigeon2(label, pigeon2).addToDashboard(this);
        return pigeon2;
    }

    /**
     * Creates a Pigeon 2.
     * @param label The label to use. Shown in the dashboard.
     * @param deviceId The ID of the Pigeon 2 on the CAN bus.
     * @param canBus Name of the CAN bus the Pigeon 2 is on.
     */
    protected Pigeon2 createPigeon2(String label, int deviceId, String canBus) {
        Pigeon2 pigeon2 = new Pigeon2(deviceId, canBus);
        new HardwareSendables.Pigeon2(label, pigeon2).addToDashboard(this);
        return pigeon2;
    }

    /**
     * Creates a Digital Input.
     * @param label The label to use. Shown in the dashboard.
     * @param channel The DIO channel for the digital input.
     */
    protected DigitalInput createDigitalInput(String label, int channel) {
        DigitalInput digitalInput = new DigitalInput(channel);
        new HardwareSendables.DigitalInput(label, digitalInput).addToDashboard(this);
        return digitalInput;
    }

    /**
     * Creates a Pneumatic Hub.
     * @param label The label to use. Shown in the dashboard.
     */
    protected PneumaticHub createPneumaticHub(String label) {
        PneumaticHub pneumaticHub = new PneumaticHub();
        new HardwareSendables.PneumaticHub(label, pneumaticHub).addToDashboard(this);
        return pneumaticHub;
    }

    /**
     * Creates a Pneumatic Hub.
     * @param label The label to use. Shown in the dashboard.
     * @param module The module number to construct.
     */
    protected PneumaticHub createPneumaticHub(String label, int module) {
        PneumaticHub pneumaticHub = new PneumaticHub(module);
        new HardwareSendables.PneumaticHub(label, pneumaticHub).addToDashboard(this);
        return pneumaticHub;
    }

    /**
     * Creates a Solenoid.
     * @param label The label to use. Shown in the dashboard.
     * @param moduleType The module type to use.
     * @param channel The channel the solenoid is on.
     */
    protected Solenoid createSolenoid(String label, PneumaticsModuleType moduleType, int channel) {
        Solenoid solenoid = new Solenoid(moduleType, channel);
        new HardwareSendables.Solenoid(label, solenoid).addToDashboard(this);
        return solenoid;
    }

    /**
     * Creates a Solenoid.
     * @param label The label to use. Shown in the dashboard.
     * @param module The module ID to use.
     * @param moduleType The module type to use.
     * @param channel The channel the solenoid is on.
     */
    protected Solenoid createSolenoid(String label, int module, PneumaticsModuleType moduleType, int channel) {
        Solenoid solenoid = new Solenoid(module, moduleType, channel);
        new HardwareSendables.Solenoid(label, solenoid).addToDashboard(this);
        return solenoid;
    }

    /**
     * Creates a Double Solenoid.
     * @param label The label to use. Shown in the dashboard.
     * @param moduleType The module type to use.
     * @param forwardChannel The forward channel on the module to control.
     * @param reverseChannel The reverse channel on the module to control.
     */
    protected DoubleSolenoid createDoubleSolenoid(String label, PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        DoubleSolenoid doubleSolenoid = new DoubleSolenoid(moduleType, forwardChannel, reverseChannel);
        new HardwareSendables.DoubleSolenoid(label, doubleSolenoid).addToDashboard(this);
        return doubleSolenoid;
    }

    /**
     * Creates a Double Solenoid.
     * @param label The label to use. Shown in the dashboard.
     * @param module The module ID to use.
     * @param moduleType The module type to use.
     * @param forwardChannel The forward channel on the module to control.
     * @param reverseChannel The reverse channel on the module to control.
     */
    protected DoubleSolenoid createDoubleSolenoid(
        String label,
        int module,
        PneumaticsModuleType moduleType,
        int forwardChannel,
        int reverseChannel
    ) {
        DoubleSolenoid doubleSolenoid = new DoubleSolenoid(module, moduleType, forwardChannel, reverseChannel);
        new HardwareSendables.DoubleSolenoid(label, doubleSolenoid).addToDashboard(this);
        return doubleSolenoid;
    }
}
