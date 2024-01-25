package org.team340.lib.swerve.config;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.ArrayList;
import java.util.List;
import java.util.MissingResourceException;
import java.util.function.Consumer;
import org.team340.lib.blacklight.BlacklightConfig;
import org.team340.lib.swerve.SwerveBase.SwerveEncoderType;
import org.team340.lib.swerve.SwerveBase.SwerveIMUType;
import org.team340.lib.swerve.SwerveBase.SwerveMotorType;
import org.team340.lib.util.config.FeedForwardConfig;
import org.team340.lib.util.config.PIDConfig;

// TODO Documentation (startup and tuning)

/**
 * Config builder for {@link SwerveBase}.
 */
public class SwerveConfig {

    private SwerveIMUType imuType;
    private Object[] imuArgs;
    private double period = -1.0;
    private PIDConfig movePID;
    private FeedForwardConfig moveFF;
    private PIDConfig turnPID;
    private double moveRampRate = -1.0;
    private double turnRampRate = -1.0;
    private double optimalVoltage = -1.0;
    private double moveCurrentLimit = -1.0;
    private double turnCurrentLimit = -1.0;
    private double moveGearRatio = -1.0;
    private double turnGearRatio = -1.0;
    private double wheelDiameterInches = -1.0;
    private double maxV = -1.0;
    private double maxRv = -1.0;
    private double maxA = -1.0;
    private double maxModuleRv = -1.0;
    private SwerveMotorType moveMotorType;
    private SwerveMotorType turnMotorType;
    private double discretizationLookahead = -1.0;
    private double[] standardDeviations;
    private double fieldLength = -1.0;
    private double fieldWidth = -1.0;
    private Config sysIdConfig = null;
    private List<SwerveModuleConfig> modules = new ArrayList<>();
    private List<BlacklightConfig> blacklights = new ArrayList<>();

    /**
     * Use an ADIS16470 IMU.
     * @param yawAxis The axis to use for yaw.
     * @param pitchAxis The axis to use for pitch.
     * @param rollAxis The axis to use for roll.
     * @param port The SPI port used.
     * @param calibrationTime The time frame to calibrate for.
     */
    public SwerveConfig useADIS16470(
        ADIS16470_IMU.IMUAxis yawAxis,
        ADIS16470_IMU.IMUAxis pitchAxis,
        ADIS16470_IMU.IMUAxis rollAxis,
        SPI.Port port,
        ADIS16470_IMU.CalibrationTime calibrationTime
    ) {
        imuType = SwerveIMUType.ADIS16470;
        imuArgs = new Object[] { yawAxis, pitchAxis, rollAxis, port, calibrationTime };
        return this;
    }

    /**
     * Use a Pigeon 2 IMU.
     * @param deviceId The device's ID on the CAN bus.
     */
    public SwerveConfig usePigeon2(int deviceId) {
        return usePigeon2(deviceId, "");
    }

    /**
     * Use a Pigeon 2 IMU.
     * @param deviceId The device's ID on the CAN bus.
     * @param canBus The name of the CAN bus being used.
     */
    public SwerveConfig usePigeon2(int deviceId, String canBus) {
        imuType = SwerveIMUType.PIGEON2;
        imuArgs = new Object[] { deviceId, canBus };
        return this;
    }

    /**
     * Gets the selected IMU's type.
     */
    public SwerveIMUType getImuType() {
        return imuType;
    }

    /**
     * Gets arguments for the selected IMU.
     */
    public Object[] getImuArgs() {
        return imuArgs;
    }

    /**
     * Sets the loop period used.
     * By default, {@code TimedRobot} uses a period of {@code 0.020} seconds.
     * @param period The loop period in seconds.
     */
    public SwerveConfig setPeriod(double period) {
        this.period = period;
        return this;
    }

    /**
     * Gets the configured loop period.
     */
    public double getPeriod() {
        return period;
    }

    /**
     * Sets PID constants for move motors.
     * A good starting point is a {@code p} value of {@code 0.001}, and {@code i} and {@code d} values of {@code 0.0}.
     * @param p Proportional gain constant.
     * @param i Integral gain constant.
     * @param d Derivative gain constant.
     * @param iZone Integral range.
     */
    public SwerveConfig setMovePID(double p, double i, double d, double iZone) {
        movePID = new PIDConfig(p, i, d, iZone);
        return this;
    }

    /**
     * Gets the configured PID constants for move motors.
     */
    public PIDConfig getMovePID() {
        return movePID;
    }

    /**
     * Sets feed forward constants for move motors. Note that turn motors don't use feed forward, as it typically causes the motor to burn out.
     * A good starting point is a {@code v} value of {@code <Optimal Voltage> / <Max Velocity>}, and {@code s} and {@code v} values of {@code 0.0}.
     * These values can be obtained via characterization using sysID.
     * @param s The static gain.
     * @param v The velocity gain.
     * @param a The acceleration gain.
     */
    public SwerveConfig setMoveFF(double s, double v, double a) {
        moveFF = new FeedForwardConfig(s, v, a);
        return this;
    }

    /**
     * Gets the configured feed forward constants for move motors.
     */
    public FeedForwardConfig getMoveFF() {
        return moveFF;
    }

    /**
     * Sets PID constants for turn motors.
     * A good starting point is a {@code p} value of {@code 0.5}, a {@code i} value of {@code 0.0}, and a {@code d} value of {@code 15.0}.
     * @param p Proportional gain constant.
     * @param i Integral gain constant.
     * @param d Derivative gain constant.
     * @param iZone Integral range.
     */
    public SwerveConfig setTurnPID(double p, double i, double d, double iZone) {
        this.turnPID = new PIDConfig(p, i, d, iZone);
        return this;
    }

    /**
     * Gets the configured PID constants for turn motors.
     */
    public PIDConfig getTurnPID() {
        return turnPID;
    }

    /**
     * Sets the motor ramp rate.
     * @param moveRampRate Time in seconds to go from {@code 0.0} to full throttle on the move motors.
     * @param turnRampRate Time in seconds to go from {@code 0.0} to full throttle on the turn motors.
     */
    public SwerveConfig setRampRate(double moveRampRate, double turnRampRate) {
        this.moveRampRate = moveRampRate;
        this.turnRampRate = turnRampRate;
        return this;
    }

    /**
     * Gets the move ramp rate in seconds.
     */
    public double getMoveRampRate() {
        return moveRampRate;
    }

    /**
     * Gets the turn ramp rate in seconds.
     */
    public double getTurnRampRate() {
        return turnRampRate;
    }

    /**
     * Sets power properties.
     * @param optimalVoltage Optimal voltage to run at. Should be {@code 12}.
     * @param moveCurrentLimit A current limit in amps for move motors. This is typically {@code 40}.
     * @param turnCurrentLimit A current limit in amps for turn motors. This is typically {@code 30}.
     */
    public SwerveConfig setPowerProperties(double optimalVoltage, double moveCurrentLimit, double turnCurrentLimit) {
        this.optimalVoltage = optimalVoltage;
        this.moveCurrentLimit = moveCurrentLimit;
        this.turnCurrentLimit = turnCurrentLimit;
        return this;
    }

    /**
     * Gets the configured optimal voltage.
     */
    public double getOptimalVoltage() {
        return optimalVoltage;
    }

    /**
     * Gets the configured current limit for move motors in amps.
     */
    public double getMoveCurrentLimit() {
        return moveCurrentLimit;
    }

    /**
     * Gets the configured current limit for turn motors in amps.
     */
    public double getTurnCurrentLimit() {
        return turnCurrentLimit;
    }

    /**
     * Sets swerve gearing properties.
     * @param moveGearRatio The move gear ratio (inverse of the gearing reduction).
     * @param turnGearRatio The turn gear ratio (inverse of the gearing reduction).
     * @param wheelDiameterInches The wheel diameter in inches.
     */
    public SwerveConfig setMechanicalProperties(double moveGearRatio, double turnGearRatio, double wheelDiameterInches) {
        this.moveGearRatio = moveGearRatio;
        this.turnGearRatio = turnGearRatio;
        this.wheelDiameterInches = wheelDiameterInches;
        return this;
    }

    /**
     * Gets the configured gear ratio for the move motors.
     */
    public double getMoveGearRatio() {
        return moveGearRatio;
    }

    /**
     * Gets the configured gear ratio for the turn motors.
     */
    public double getTurnGearRatio() {
        return turnGearRatio;
    }

    /**
     * Gets the configured wheel diameter.
     */
    public double getWheelDiameterInches() {
        return wheelDiameterInches;
    }

    /**
     * Sets speed constraints.
     * You may find more predictable behavior by setting these values lower than the actual maximum capabilities of your robot.
     * It is recommended that these values are tested for using an actual robot. An easy way to do so is to set these values to an impossibly high value, then examine the outputs in network tables.
     * Initial theoretical values can be estimated using the following formulas:
     *
     * <br><br>
     * <b>Max Robot Velocity:</b> {@code (<Move Motor Free Speed RPM> * 0.80 / 60) / (<Move Gear Ratio> / (<Wheel Diameter (Meters)> * PI))}
     *
     * <br><br>
     * <b>Max Robot Acceleration:</b> {@code <Max Robot Velocity> * 2} (VERY much an estimate, typical ballpark acceleration for robots weighing ~120 pounds)
     * Can also be pulled from Choreo.
     *
     * <br><br>
     * <b>Max Robot Rotational Velocity:</b> {@code (<Max Robot Velocity> / (<Robot Length (Meters)>^2 + <Robot Width (Meters)>^2)^0.5) * 2}
     *
     * <br><br>
     * <b>Max Module Rotational Velocity:</b> {@code (<Turn Motor Free Speed RPM> / 60) / (<Turn Gear Ratio> / (PI * 2)) * 0.7}
     *
     * @param maxV The maximum velocity the robot is capable of in meters/second.
     * @param maxRv The maximum rotational velocity the robot is capable of in radians/second.
     * @param maxA The maximum acceleration the robot is capable of in meters/second/second.
     * @param maxModuleRv The maximum rotational acceleration of a single swerve module in radians/second/second.
     */
    public SwerveConfig setSpeedConstraints(double maxV, double maxRv, double maxA, double maxModuleRv) {
        this.maxV = maxV;
        this.maxRv = maxRv;
        this.maxA = maxA;
        this.maxModuleRv = maxModuleRv;
        return this;
    }

    /**
     * Gets the configured maximum robot velocity in meters/second.
     */
    public double getMaxV() {
        return maxV;
    }

    /**
     * Gets the configured maximum robot rotational velocity in radians/second.
     */
    public double getMaxRv() {
        return maxRv;
    }

    /**
     * Gets the configured maximum robot acceleration in meters/second/second.
     */
    public double getMaxA() {
        return maxA;
    }

    /**
     * Gets the configured maximum module rotational velocity in radians/second.
     */
    public double getMaxModuleRv() {
        return maxModuleRv;
    }

    /**
     * Sets the motor types used.
     * @param moveMotorType The move motor type.
     * @param turnMotorType The turn motor type.
     */
    public SwerveConfig setMotorTypes(SwerveMotorType moveMotorType, SwerveMotorType turnMotorType) {
        this.moveMotorType = moveMotorType;
        this.turnMotorType = turnMotorType;
        return this;
    }

    /**
     * Gets the move motor type.
     */
    public SwerveMotorType getMoveMotorType() {
        return moveMotorType;
    }

    /**
     * Gets the turn motor type.
     */
    public SwerveMotorType getTurnMotorType() {
        return turnMotorType;
    }

    /**
     * Sets the discretization lookahead in seconds.
     * Used for countering drift caused by translating and rotating simultaneously. This
     * should be characterized on the robot, as it effectively models lag in the physical
     * swerve system which is difficult to simulate. A good starting value is around double
     * the robot's loop period. Values below the robot's loop period is not recommended.
     * @param discretizationLookahead Lookahead in seconds.
     */
    public SwerveConfig setDiscretizationLookahead(double discretizationLookahead) {
        this.discretizationLookahead = discretizationLookahead;
        return this;
    }

    /**
     * Gets the configured discretization lookahead in seconds.
     */
    public double getDiscretizationLookahead() {
        return discretizationLookahead;
    }

    /**
     * Sets the standard deviations for pose estimation from module odometry.
     * A good starting configuration is all axis with a magnitude of {@code 0.1}.
     * @param x The X axis standard deviation in meters.
     * @param y The Y axis standard deviation in meters.
     * @param rot The rotational standard deviation in radians.
     */
    public SwerveConfig setStandardDeviations(double x, double y, double rot) {
        this.standardDeviations = new double[] { x, y, rot };
        return this;
    }

    /**
     * Gets the configured standard deviations for odometry, as an array of {@code [x, y, rot]}.
     */
    public double[] getStandardDeviations() {
        return standardDeviations;
    }

    /**
     * Sets the field size.
     * @param fieldLength The field's length in meters. Typically {@code 16.5417}.
     * @param fieldWidth The field's width in meters. Typically {@code 8.0136}.
     */
    public SwerveConfig setFieldSize(double fieldLength, double fieldWidth) {
        this.fieldLength = fieldLength;
        this.fieldWidth = fieldWidth;
        return this;
    }

    /**
     * Gets the configured field length in meters.
     */
    public double getFieldLength() {
        return fieldLength;
    }

    /**
     * Gets the configured field width in meters.
     */
    public double getFieldWidth() {
        return fieldWidth;
    }

    public SwerveConfig setSysIdConfig(Config sysIdConfig) {
        this.sysIdConfig = sysIdConfig;
        return this;
    }

    public Config getSysIdConfig() {
        return sysIdConfig;
    }

    /**
     * Adds a module to the config.
     * See {@link SwerveModuleConfig}.
     * @param moduleConfig The config of module to add.
     */
    public SwerveConfig addModule(SwerveModuleConfig moduleConfig) {
        modules.add(moduleConfig);
        return this;
    }

    /**
     * Adds a module to the config.
     * See {@link SwerveModuleConfig}.
     * @param moduleConfigConsumer A consumer for the module config.
     */
    public SwerveConfig addModule(Consumer<SwerveModuleConfig> moduleConfigConsumer) {
        SwerveModuleConfig moduleConfig = new SwerveModuleConfig();
        moduleConfigConsumer.accept(moduleConfig);
        modules.add(moduleConfig);
        return this;
    }

    /**
     * Gets the configured modules.
     */
    public List<SwerveModuleConfig> getModules() {
        return modules;
    }

    /**
     * Adds a Blacklight to the config.
     * See {@link BlacklightConfig}.
     * @param blacklightConfig The config of the Blacklight to add.
     */
    public SwerveConfig addBlacklight(BlacklightConfig blacklightConfig) {
        blacklights.add(blacklightConfig);
        return this;
    }

    /**
     * Adds a Blacklight to the config.
     * See {@link BlacklightConfig}.
     * @param blacklightConfigConsumer A consumer for the Blacklight config.
     */
    public SwerveConfig addBlacklight(Consumer<BlacklightConfig> blacklightConfigConsumer) {
        BlacklightConfig blacklightConfig = new BlacklightConfig();
        blacklightConfigConsumer.accept(blacklightConfig);
        blacklights.add(blacklightConfig);
        return this;
    }

    /**
     * Gets configured Blacklights.
     */
    public List<BlacklightConfig> getBlacklights() {
        return blacklights;
    }

    /**
     * Verifies the config as well as the config's modules.
     * Throws an error if an issue is found.
     */
    public void verify() {
        if (imuType == null) throwMissing("IMU");
        if (imuArgs == null) throwMissing("IMU Args");
        if (period == -1) throwMissing("Period");
        if (movePID == null) throwMissing("Move PID");
        if (moveFF == null) throwMissing("Move FF");
        if (turnPID == null) throwMissing("Turn PID");
        if (moveRampRate == -1) throwMissing("MoveRamp Rate");
        if (turnRampRate == -1) throwMissing("Turn Ramp Rate");
        if (optimalVoltage == -1) throwMissing("Optimal Voltage");
        if (moveCurrentLimit == -1) throwMissing("Move Current Limit");
        if (turnCurrentLimit == -1) throwMissing("Turn Current Limit");
        if (moveGearRatio == -1) throwMissing("Move Gear Ratio");
        if (turnGearRatio == -1) throwMissing("Turn Gear Ratio");
        if (wheelDiameterInches == -1) throwMissing("Wheel Diameter");
        if (maxV == -1) throwMissing("Max Robot Velocity");
        if (maxRv == -1) throwMissing("Max Robot Rotational Velocity");
        if (maxA == -1) throwMissing("Max Robot Acceleration");
        if (maxModuleRv == -1) throwMissing("Max Module Rotational Velocity");
        if (moveMotorType == null) throwMissing("Move Motor Type");
        if (turnMotorType == null) throwMissing("Turn Motor Type");
        if (discretizationLookahead == -1) throwMissing("Discretization Lookahead");
        if (standardDeviations == null) throwMissing("Standard Deviations");
        if (fieldLength == -1) throwMissing("Field Length");
        if (fieldWidth == -1) throwMissing("Field Width");
        if (sysIdConfig == null) throwMissing("SysId Config");
        if (modules.size() == 0) throwMissing("Modules");

        for (SwerveModuleConfig module : modules) {
            module.verify();

            if (
                (
                    !turnMotorType.equals(SwerveMotorType.SPARK_MAX_BRUSHED) &&
                    !turnMotorType.equals(SwerveMotorType.SPARK_MAX_BRUSHLESS) &&
                    !turnMotorType.equals(SwerveMotorType.SPARK_FLEX_BRUSHED) &&
                    !turnMotorType.equals(SwerveMotorType.SPARK_FLEX_BRUSHLESS)
                ) &&
                module.getEncoderType().equals(SwerveEncoderType.SPARK_ENCODER)
            ) throw new UnsupportedOperationException("Cannot use Spark attached encoder on non-Spark motor");

            if (
                !module.getMoveMotorCanBus().isEmpty() && !moveMotorType.equals(SwerveMotorType.TALONFX)
            ) throw new UnsupportedOperationException("Cannot set custom CAN bus for non-Talon FX motor");
            if (
                !module.getTurnMotorCanBus().isEmpty() && !turnMotorType.equals(SwerveMotorType.TALONFX)
            ) throw new UnsupportedOperationException("Cannot set custom CAN bus for non-Talon FX motor");
        }
    }

    private void throwMissing(String key) {
        throw new MissingResourceException("Missing value: " + key, this.getClass().getSimpleName(), key);
    }
}
