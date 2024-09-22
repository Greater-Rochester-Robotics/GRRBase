package org.team340.lib.swerve.config;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.MissingResourceException;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveIMUs.SwerveIMU;

/**
 * Config builder for {@link SwerveBase}.
 */
public class SwerveConfig {

    private double period = -1.0;
    private double odometryPeriod = -1.0;
    private double discretizationPeriod = -1.0;
    private double fieldLength = -1.0;
    private double fieldWidth = -1.0;
    private double[] movePID;
    private double[] moveFF;
    private double[] turnPID;
    private boolean moveBrakeMode = false;
    private boolean turnBrakeMode = false;
    private double velocity = -1.0;
    private double rotVelocity = -1.0;
    private double acceleration = -1.0;
    private double moduleRotVel = -1.0;
    private double voltage = -1.0;
    private double moveCurrentLimit = -1.0;
    private double turnCurrentLimit = -1.0;
    private double moveGearRatio = -1.0;
    private double turnGearRatio = -1.0;
    private double couplingRatio = -1.0;
    private double wheelDiameter = -1.0;
    private double[] odometryStd;
    private double[] visionStd;
    private SwerveIMU.Ctor imu;
    private String phoenixCanBus = "";
    private boolean phoenixPro = false;
    private boolean phoenixMoveFOC = false;
    private boolean phoenixTurnFOC = false;
    private SwerveModuleConfig[] modules;

    /**
     * Sets various timings utilized by the robot.
     * @param period The robot's main loop period in seconds.
     * @param odometry The period to update odometry in seconds. Odometry can be optionally disabled
     * @param discretization The period to look ahead for discretizing chassis speeds in seconds.
     */
    public SwerveConfig setTimings(double period, double odometry, double discretization) {
        this.period = period;
        odometryPeriod = odometry;
        discretizationPeriod = discretization;
        return this;
    }

    /**
     * Gets the configured loop period in seconds.
     */
    public double getPeriod() {
        return period;
    }

    /**
     * Gets period in seconds between odometry samples.
     */
    public double getOdometryPeriod() {
        return odometryPeriod;
    }

    /**
     * Gets the configured discretization lookahead period in seconds.
     */
    public double getDiscretizationPeriod() {
        return discretizationPeriod;
    }

    /**
     * Sets the field size.
     * @param length The field's length in meters.
     * @param width The field's width in meters.
     */
    public SwerveConfig setFieldSize(double length, double width) {
        fieldLength = length;
        fieldWidth = width;
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
     * Sets PID gains for move motors. Note that {@link TalonFX} motors do not support {@code iZone}.
     * @param kP Proportional gain constant.
     * @param kI Integral gain constant.
     * @param kD Derivative gain constant.
     * @param iZone Integral range.
     */
    public SwerveConfig setMovePID(double kP, double kI, double kD, double iZone) {
        movePID = new double[] { kP, kI, kD, iZone };
        return this;
    }

    /**
     * Gets the configured PID gains for move motors, as a
     * tuple of {@code [kP, kI, kD, iZone]}.
     */
    public double[] getMovePID() {
        return movePID;
    }

    /**
     * Sets feed forward constants for move motors. A good starting point is a {@code kV} value
     * of {@code <Optimal Voltage> / <Max Velocity>}, and {@code kS} and {@code kV} values of
     * {@code 0.0}. These values can be obtained via characterization using sysID.
     * @param kS The static gain.
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     */
    public SwerveConfig setMoveFF(double kS, double kV, double kA) {
        moveFF = new double[] { kS, kV, kA };
        return this;
    }

    /**
     * Gets the configured feed forward gains for move motors,
     * as a tuple of {@code [kS, kV, kA]}.
     */
    public double[] getMoveFF() {
        return moveFF;
    }

    /**
     * Sets PID gains for turn motors. Note that {@link TalonFX} motors do not support {@code iZone}.
     * @param kP Proportional gain constant.
     * @param kI Integral gain constant.
     * @param kD Derivative gain constant.
     * @param iZone Integral range.
     */
    public SwerveConfig setTurnPID(double kP, double kI, double kD, double iZone) {
        this.turnPID = new double[] { kP, kI, kD, iZone };
        return this;
    }

    /**
     * Gets the configured PID gains for turn motors, as a
     * tuple of {@code [kP, kI, kD, iZone]}.
     */
    public double[] getTurnPID() {
        return movePID;
    }

    /**
     * Sets motor brake modes.
     * @param move If the move motors should have brake mode enabled.
     * @param turn If the turn motors should have brake mode enabled.
     */
    public SwerveConfig setBrakeMode(boolean move, boolean turn) {
        moveBrakeMode = move;
        turnBrakeMode = turn;
        return this;
    }

    /**
     * Gets the configured brake mode for move motors.
     */
    public boolean getMoveBrakeMode() {
        return moveBrakeMode;
    }

    /**
     * Gets the configured brake mode for turn motors.
     */
    public boolean getTurnBrakeMode() {
        return turnBrakeMode;
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
     * @param rotVelocity The maximum rotational velocity the robot is capable of in radians/second.
     */
    public SwerveConfig setMaxSpeeds(double velocity, double rotVelocity) {
        this.velocity = velocity;
        this.rotVelocity = rotVelocity;
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
    public double getRotVelocity() {
        return rotVelocity;
    }

    /**
     * Sets constraints for the ratelimiter.
     *
     * <br><br>
     * You may find more predictable behavior by setting these values slightly lower than the actual maximum capabilities of your robot.
     * It is recommended that these values are found empirically using an actual robot. An easy way to do so is to configure infeasible ratelimits, then analyze telemetry.
     *
     * @param acceleration The maximum acceleration the robot is capable of in meters/second/second.
     * @param moduleRotVel The maximum module rotational velocity the robot is capable of in radians/second.
     */
    public SwerveConfig setRatelimits(double acceleration, double moduleRotVel) {
        this.acceleration = acceleration;
        this.moduleRotVel = moduleRotVel;
        return this;
    }

    /**
     * Gets the configured maximum robot acceleration in meters/second/second.
     */
    public double getAcceleration() {
        return acceleration;
    }

    /**
     * Gets the configured maximum module rotational velocity in radians/second.
     */
    public double getModuleRotVel() {
        return moduleRotVel;
    }

    /**
     * Sets power properties.
     * @param voltage Optimal voltage to run at. Should be {@code 12}.
     * @param moveCurrentLimit A current limit in amps for move motors. This is typically {@code 40}.
     * @param turnCurrentLimit A current limit in amps for turn motors. This is typically {@code 30}.
     */
    public SwerveConfig setPowerProperties(double voltage, double moveCurrentLimit, double turnCurrentLimit) {
        this.voltage = voltage;
        this.moveCurrentLimit = moveCurrentLimit;
        this.turnCurrentLimit = turnCurrentLimit;
        return this;
    }

    /**
     * Gets the configured optimal voltage.
     */
    public double getVoltage() {
        return voltage;
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
     * @param moveRatio The move gear ratio (inverse of the gearing reduction).
     * @param turnRatio The turn gear ratio (inverse of the gearing reduction).
     * @param couplingRatio The ratio between the wheel and the module's angle. Used as a compensation factor, set to {@code 0.0} to disable.
     * @param wheelDiameter The wheel diameter in meters.
     */
    public SwerveConfig setMechanicalProperties(double moveRatio, double turnRatio, double couplingRatio, double wheelDiameter) {
        moveGearRatio = moveRatio;
        turnGearRatio = turnRatio;
        this.couplingRatio = couplingRatio;
        this.wheelDiameter = wheelDiameter;
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
     * Gets the configured coupling ratio.
     */
    public double getCouplingRatio() {
        return couplingRatio;
    }

    /**
     * Gets the configured wheel diameter in meters.
     */
    public double getWheelDiameter() {
        return wheelDiameter;
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
     * Gets the configured standard deviations for odometry, as a tuple of {@code [x, y, rot]}.
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
     * Gets the configured standard deviations for vision, as a tuple of {@code [x, y, rot]}.
     */
    public double[] getVisionStd() {
        return visionStd;
    }

    /**
     * Sets the IMU to use.
     * @param imu The IMU to use, generated by the {@link SwerveIMUs} class.
     */
    public SwerveConfig setIMU(SwerveIMU.Ctor imu) {
        this.imu = imu;
        return this;
    }

    /**
     * Gets the configured IMU supplier.
     */
    public SwerveIMU.Ctor getIMU() {
        return imu;
    }

    /**
     * Sets available features for Phoenix devices. For Phoenix Pro features to be utilized by the Swerve API,
     * all devices must be activated, and intrinsically must all be Phoenix devices (Pigeon2 + TalonFX + CANcoder).
     * This configuration option can be omitted safely if its features are not needed.
     * @param canBus Name of the CAN bus utilized by Phoenix devices.
     * @param pro {@code true} if Phoenix Pro is active on <i>all</i> devices used by the swerve API.
     * @param moveFOC If move motors should enable FOC. {@code pro} can still be {@code false} while enabling FOC, provided all move motors are still licensed.
     * @param turnFOC If turn motors should enable FOC. {@code pro} can still be {@code false} while enabling FOC, provided all turn motors are still licensed.
     */
    public SwerveConfig setPhoenixFeatures(String canBus, boolean pro, boolean moveFOC, boolean turnFOC) {
        phoenixCanBus = canBus;
        phoenixPro = pro;
        phoenixMoveFOC = moveFOC;
        phoenixTurnFOC = turnFOC;
        return this;
    }

    /**
     * Gets the name of the CAN bus utilized by Phoenix devices.
     */
    public String getPhoenixCanBus() {
        return phoenixCanBus;
    }

    /**
     * Returns {@code true} if Phoenix Pro is available on <i>all</i> devices used by the swerve API.
     */
    public boolean getPhoenixPro() {
        return phoenixPro;
    }

    /**
     * Returns {@code true} if Phoenix move motors are configured to enable FOC control.
     */
    public boolean getPhoenixMoveFOC() {
        return phoenixMoveFOC;
    }

    /**
     * Returns {@code true} if Phoenix turn motors are configured to enable FOC control.
     */
    public boolean getPhoenixTurnFOC() {
        return phoenixTurnFOC;
    }

    /**
     * Sets module configs.
     * See {@link SwerveModuleConfig}.
     * @param modules The configs of the modules to add.
     */
    public SwerveConfig setModules(SwerveModuleConfig... modules) {
        this.modules = modules;
        return this;
    }

    /**
     * Gets the configured modules.
     */
    public SwerveModuleConfig[] getModules() {
        return modules;
    }

    /**
     * Verifies the config as well as the config's modules.
     * Throws an error if an issue is found.
     */
    public void verify() {
        if (period == -1.0) throwMissing("Period");
        if (odometryPeriod == -1.0) throwMissing("Odometry Period");
        if (discretizationPeriod == -1.0) throwMissing("Discretization Period");
        if (fieldLength == -1.0) throwMissing("Field Length");
        if (fieldWidth == -1.0) throwMissing("Field Width");
        if (movePID == null) throwMissing("Move PID");
        if (moveFF == null) throwMissing("Move FF");
        if (turnPID == null) throwMissing("Turn PID");
        if (velocity == -1.0) throwMissing("Velocity");
        if (rotVelocity == -1.0) throwMissing("Rotational Velocity");
        if (acceleration == -1.0) throwMissing("Acceleration");
        if (moduleRotVel == -1.0) throwMissing("Module Rotational Velocity");
        if (voltage == -1.0) throwMissing("Voltage");
        if (moveCurrentLimit == -1.0) throwMissing("Move Current Limit");
        if (turnCurrentLimit == -1.0) throwMissing("Turn Current Limit");
        if (moveGearRatio == -1.0) throwMissing("Move Gear Ratio");
        if (turnGearRatio == -1.0) throwMissing("Turn Gear Ratio");
        if (couplingRatio == -1.0) throwMissing("Coupling Ratio");
        if (wheelDiameter == -1.0) throwMissing("Wheel Diameter");
        if (odometryStd == null) throwMissing("Odometry Standard Deviations");
        if (visionStd == null) throwMissing("Vision Standard Deviations");
        if (imu == null) throwMissing("IMU");
        if (modules == null) throwMissing("Modules");

        for (SwerveModuleConfig module : modules) {
            module.verify();
        }
    }

    private void throwMissing(String key) {
        throw new MissingResourceException("Missing value: " + key, this.getClass().getSimpleName(), key);
    }
}
