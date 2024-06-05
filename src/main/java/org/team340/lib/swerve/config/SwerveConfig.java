package org.team340.lib.swerve.config;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.util.ArrayList;
import java.util.List;
import java.util.MissingResourceException;
import java.util.function.Consumer;
import org.team340.lib.swerve.SwerveBase;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.config.FeedForwardConfig;
import org.team340.lib.util.config.PIDConfig;

/**
 * Config builder for {@link SwerveBase}.
 */
public class SwerveConfig {

    private SwerveIMU.Type imuType;
    private Object[] imuArgs;
    private double period = -1.0;
    private PIDConfig movePID;
    private FeedForwardConfig moveFF;
    private PIDConfig turnPID;
    private double moveRampRate = -1.0;
    private double turnRampRate = -1.0;
    private SwerveMotor.Type moveMotorType;
    private SwerveMotor.Type turnMotorType;
    private double velocity = -1.0;
    private double rotationalVelocity = -1.0;
    private double acceleration = -1.0;
    private double deceleration = -1.0;
    private double moduleRotationalVelocity = -1.0;
    private double trajectoryVelocity = -1.0;
    private double trajectoryAcceleration = -1.0;
    private double optimalVoltage = -1.0;
    private double moveCurrentLimit = -1.0;
    private double turnCurrentLimit = -1.0;
    private double moveGearRatio = -1.0;
    private double turnGearRatio = -1.0;
    private double wheelDiameterInches = -1.0;
    private double discretizationLookahead = -1.0;
    private double odometryPeriod = -1.0;
    private double[] odometryStd;
    private double[] visionStd;
    private Config sysIdConfig = null;
    private double fieldLength = -1.0;
    private double fieldWidth = -1.0;
    private List<SwerveModuleConfig> modules = new ArrayList<>();

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
        imuType = SwerveIMU.Type.ADIS16470;
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
        imuType = SwerveIMU.Type.PIGEON2;
        imuArgs = new Object[] { deviceId, canBus };
        return this;
    }

    /**
     * Gets the selected IMU's type.
     */
    public SwerveIMU.Type getImuType() {
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
     * Increase this to reduce the load on the motors.
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
     * Sets the motor types used.
     * @param moveMotorType The move motor type.
     * @param turnMotorType The turn motor type.
     */
    public SwerveConfig setMotorTypes(SwerveMotor.Type moveMotorType, SwerveMotor.Type turnMotorType) {
        this.moveMotorType = moveMotorType;
        this.turnMotorType = turnMotorType;
        return this;
    }

    /**
     * Gets the move motor type.
     */
    public SwerveMotor.Type getMoveMotorType() {
        return moveMotorType;
    }

    /**
     * Gets the turn motor type.
     */
    public SwerveMotor.Type getTurnMotorType() {
        return turnMotorType;
    }

    /**
     * Sets max speed constraints.
     * These are used for constraining the requested velocity commanded to swerve modules, as well as scaling when driving by a percent of max speed.
     *
     * <br><br>
     * You may find more predictable behavior by setting these values slightly lower than the actual maximum capabilities of your robot.
     * It is recommended that these values are found empirically using an actual robot. An easy way to do so is to configure infeasible ratelimits, then analyze telemetry.
     * Initial theoretical values can be estimated using the following formulas:
     *
     * <br><br>
     * <b>Max Robot Velocity:</b> {@code (<Move Motor Free Speed RPM> * 0.80 / 60) / (<Move Gear Ratio> / (<Wheel Diameter (Meters)> * PI))}
     *
     * <br><br>
     * <b>Max Robot Rotational Velocity:</b> {@code (<Max Robot Velocity> / (<Robot Length (Meters)>^2 + <Robot Width (Meters)>^2)^0.5) * 2}
     *
     * @param velocity The maximum velocity the robot is capable of in meters/second.
     * @param rotationalVelocity The maximum rotational velocity the robot is capable of in radians/second.
     */
    public SwerveConfig setMaxSpeeds(double velocity, double rotationalVelocity) {
        this.velocity = velocity;
        this.rotationalVelocity = rotationalVelocity;
        return this;
    }

    /**
     * Gets the configured maximum robot velocity in meters/second.
     */
    public double getVelocity() {
        return velocity;
    }

    /**
     * Gets the configured maximum robot rotational velocity in radians/second.
     */
    public double getRotationalVelocity() {
        return rotationalVelocity;
    }

    /**
     * Sets constraints for the ratelimiter.
     *
     * <br><br>
     * You may find more predictable behavior by setting these values slightly lower than the actual maximum capabilities of your robot.
     * It is recommended that these values are found empirically using an actual robot. An easy way to do so is to configure infeasible ratelimits, then analyze telemetry.
     *
     * @param acceleration The maximum acceleration the robot is capable of in meters/second/second.
     * @param deceleration The maximum deceleration the robot is capable of in meters/second/second (positive).
     * @param moduleRotationalVelocity The maximum module rotational velocity the robot is capable of in radians/second.
     */
    public SwerveConfig setRatelimits(double acceleration, double deceleration, double moduleRotationalVelocity) {
        this.acceleration = acceleration;
        this.deceleration = deceleration;
        this.moduleRotationalVelocity = moduleRotationalVelocity;
        return this;
    }

    /**
     * Gets the configured maximum robot acceleration in meters/second/second.
     */
    public double getAcceleration() {
        return acceleration;
    }

    /**
     * Gets the configured maximum robot deceleration in meters/second/second.
     */
    public double getDeceleration() {
        return deceleration;
    }

    /**
     * Gets the configured maximum module rotational velocity in radians/second.
     */
    public double getModuleRotationalVelocity() {
        return moduleRotationalVelocity;
    }

    /**
     * Sets constraints for trajectory generation.
     * @param trajectoryVelocity Max trajectory velocity in meters/second.
     * @param trajectoryAcceleration Max trajectory acceleration in meters/second/second.
     */
    public SwerveConfig setTrajectoryConstraints(double trajectoryVelocity, double trajectoryAcceleration) {
        this.trajectoryVelocity = trajectoryVelocity;
        this.trajectoryAcceleration = trajectoryAcceleration;
        return this;
    }

    /**
     * Gets the configured maximum trajectory velocity in meters/second.
     */
    public double getTrajectoryVelocity() {
        return trajectoryVelocity;
    }

    /**
     * Gets te configured maximum trajectory acceleration in meters/second/second.
     */
    public double getTrajectoryAcceleration() {
        return trajectoryAcceleration;
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
     * Sets period in seconds between odometry samples.
     * @param odometryPeriod Period in seconds.
     */
    public SwerveConfig setOdometryPeriod(double odometryPeriod) {
        this.odometryPeriod = odometryPeriod;
        return this;
    }

    /**
     * Gets period in seconds between odometry samples.
     */
    public double getOdometryPeriod() {
        return odometryPeriod;
    }

    /**
     * Sets the standard deviations for pose estimation from module odometry.
     * A good starting configuration is all axis with a magnitude of {@code 0.1}.
     * @param x The X axis standard deviation in meters.
     * @param y The Y axis standard deviation in meters.
     * @param rot The rotational standard deviation in radians.
     */
    public SwerveConfig setOdometryStd(double x, double y, double rot) {
        this.odometryStd = new double[] { x, y, rot };
        return this;
    }

    /**
     * Gets the configured standard deviations for odometry, as an array of {@code [x, y, rot]}.
     */
    public double[] getOdometryStd() {
        return odometryStd;
    }

    /**
     * Sets the standard deviations for pose estimation from vision.
     * A good starting configuration is all axis with a magnitude of {@code 0.1}.
     * @param x The X axis standard deviation in meters.
     * @param y The Y axis standard deviation in meters.
     * @param rot The rotational standard deviation in radians.
     */
    public SwerveConfig setVisionStd(double x, double y, double rot) {
        this.visionStd = new double[] { x, y, rot };
        return this;
    }

    /**
     * Gets the configured standard deviations for vision, as an array of {@code [x, y, rot]}.
     */
    public double[] getVisionStd() {
        return visionStd;
    }

    /**
     * Sets config for SysId.
     */
    public SwerveConfig setSysIdConfig(Config sysIdConfig) {
        this.sysIdConfig = sysIdConfig;
        return this;
    }

    /**
     * Gets config for SysId.
     */
    public Config getSysIdConfig() {
        return sysIdConfig;
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
     * Verifies the config as well as the config's modules.
     * Throws an error if an issue is found.
     */
    public void verify() {
        if (imuType == null) throwMissing("IMU");
        if (imuArgs == null) throwMissing("IMU Args");
        if (period == -1.0) throwMissing("Period");
        if (movePID == null) throwMissing("Move PID");
        if (moveFF == null) throwMissing("Move FF");
        if (turnPID == null) throwMissing("Turn PID");
        if (moveRampRate == -1.0) throwMissing("MoveRamp Rate");
        if (turnRampRate == -1.0) throwMissing("Turn Ramp Rate");
        if (moveMotorType == null) throwMissing("Move Motor Type");
        if (turnMotorType == null) throwMissing("Turn Motor Type");
        if (velocity == -1.0) throwMissing("Velocity");
        if (rotationalVelocity == -1.0) throwMissing("Rotational Velocity");
        if (acceleration == -1.0) throwMissing("Acceleration");
        if (deceleration == -1.0) throwMissing("Deceleration");
        if (moduleRotationalVelocity == -1.0) throwMissing("Module Rotational Velocity");
        if (trajectoryVelocity == -1.0) throwMissing("Trajectory Velocity");
        if (trajectoryAcceleration == -1.0) throwMissing("Trajectory Acceleration");
        if (optimalVoltage == -1.0) throwMissing("Optimal Voltage");
        if (moveCurrentLimit == -1.0) throwMissing("Move Current Limit");
        if (turnCurrentLimit == -1.0) throwMissing("Turn Current Limit");
        if (moveGearRatio == -1.0) throwMissing("Move Gear Ratio");
        if (turnGearRatio == -1.0) throwMissing("Turn Gear Ratio");
        if (wheelDiameterInches == -1.0) throwMissing("Wheel Diameter");
        if (discretizationLookahead == -1.0) throwMissing("Discretization Lookahead");
        if (odometryPeriod == -1.0) throwMissing("Odometry Period");
        if (odometryStd == null) throwMissing("Odometry Standard Deviations");
        if (visionStd == null) throwMissing("Vision Standard Deviations");
        if (sysIdConfig == null) throwMissing("SysId Config");
        if (fieldLength == -1.0) throwMissing("Field Length");
        if (fieldWidth == -1.0) throwMissing("Field Width");
        if (modules.size() == 0) throwMissing("Modules");

        for (SwerveModuleConfig module : modules) {
            module.verify();

            if (
                (
                    !turnMotorType.equals(SwerveMotor.Type.SPARK_MAX_BRUSHED) &&
                    !turnMotorType.equals(SwerveMotor.Type.SPARK_MAX_BRUSHLESS) &&
                    !turnMotorType.equals(SwerveMotor.Type.SPARK_FLEX_BRUSHED) &&
                    !turnMotorType.equals(SwerveMotor.Type.SPARK_FLEX_BRUSHLESS)
                ) &&
                module.getEncoderType().equals(SwerveEncoder.Type.SPARK_ENCODER)
            ) throw new UnsupportedOperationException("Cannot use Spark attached encoder on non-Spark motor");

            if (
                !module.getMoveMotorCanBus().isEmpty() && !moveMotorType.equals(SwerveMotor.Type.TALONFX)
            ) throw new UnsupportedOperationException("Cannot set custom CAN bus for non-Talon FX motor");
            if (
                !module.getTurnMotorCanBus().isEmpty() && !turnMotorType.equals(SwerveMotor.Type.TALONFX)
            ) throw new UnsupportedOperationException("Cannot set custom CAN bus for non-Talon FX motor");
        }
    }

    private void throwMissing(String key) {
        throw new MissingResourceException("Missing value: " + key, this.getClass().getSimpleName(), key);
    }
}
