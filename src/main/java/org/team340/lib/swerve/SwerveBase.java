package org.team340.lib.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import java.util.ArrayList;
import java.util.List;
import org.team340.lib.GRRDashboard;
import org.team340.lib.blacklight.Blacklight;
import org.team340.lib.blacklight.BlacklightConfig;
import org.team340.lib.drivers.ADIS16470;
import org.team340.lib.math.Math2;
import org.team340.lib.subsystem.GRRSubsystem;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveAbsoluteEncoder;
import org.team340.lib.swerve.hardware.encoders.vendors.SwerveCANcoder;
import org.team340.lib.swerve.hardware.encoders.vendors.SwerveSparkMaxAttachedEncoder;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;
import org.team340.lib.swerve.hardware.imu.vendors.SwerveADIS16470;
import org.team340.lib.swerve.hardware.imu.vendors.SwervePigeon2;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.swerve.hardware.motors.vendors.SwerveSparkMax;
import org.team340.lib.swerve.hardware.motors.vendors.SwerveTalonFX;
import org.team340.lib.swerve.simulation.SwerveSimIMU;
import org.team340.lib.swerve.simulation.SwerveSimModule;
import org.team340.lib.util.SendableFactory;
import org.team340.lib.util.StringUtil;

/**
 * A general implementation of swerve drive.
 * This class consists of protected members that must be exposed in a subclassed implementation.
 *
 * Supported motors:
 * <ul>
 *   <li>Brushed Spark Max</li>
 *   <li>Brushless Spark Max</li>
 *   <li>Talon FX</li>
 * </ul>
 *
 * Supported encoders:
 * <ul>
 *   <li>CANcoder</li>
 *   <li>Spark Max Attached Encoders (Through Bore, MagEncoder with adapter board, CANandcoder, etc)</li>
 * </ul>
 *
 * Supported IMUs:
 * <ul>
 *   <li>ADIS16470 (Using modified implementation with support for 3 axis)</li>
 *   <li>Pigeon 2</li>
 * </ul>
 */
public abstract class SwerveBase extends GRRSubsystem {

    /**
     * Supported IMUs.
     */
    public static enum SwerveIMUType {
        ADIS16470,
        PIGEON2,
    }

    /**
     * Supported motors.
     */
    public static enum SwerveMotorType {
        SPARK_MAX_BRUSHED,
        SPARK_MAX_BRUSHLESS,
        TALONFX,
    }

    /**
     * Supported encoders.
     */
    public static enum SwerveAbsoluteEncoderType {
        CANCODER,
        SPARK_MAX_ATTACHED,
    }

    /**
     * The general config.
     */
    protected final SwerveConfig config;
    /**
     * Converted config measurements.
     */
    protected final SwerveConversions conversions;
    /**
     * The IMU in use.
     */
    protected final SwerveIMU imu;
    /**
     * Swerve modules.
     */
    protected final SwerveModule[] modules;
    /**
     * An array representing the positions of modules as translations from the robot's center.
     */
    protected final Translation2d[] moduleTranslations;
    /**
     * The kinematics instance in use.
     */
    protected final SwerveDriveKinematics kinematics;
    /**
     * The chassis speeds factory.
     */
    protected final SwerveSpeedsFactory speedsFactory;
    /**
     * The target controller.
     */
    protected final SwerveTargetController targetController;
    /**
     * The swerve's representation on a field.
     */
    protected final SwerveField2d field;
    /**
     * The pose estimator in use.
     */
    protected final SwerveDrivePoseEstimator poseEstimator;
    /**
     * Blacklights used by the swerve subsystem.
     */
    protected final List<Blacklight> blacklights = new ArrayList<>();

    /**
     * Create the GRRSwerve subsystem.
     * @param label The label for the subsystem. Shown in the dashboard.
     * @param config Swerve config, use {@link SwerveConfig} as a builder for generating configs.
     */
    public SwerveBase(String label, SwerveConfig config) {
        super(label);
        config.verify();
        this.config = config;

        imu = createIMU();

        modules = new SwerveModule[config.getModules().size()];
        moduleTranslations = new Translation2d[modules.length];
        for (int i = 0; i < config.getModules().size(); i++) {
            SwerveModuleConfig moduleConfig = config.getModules().get(i);
            modules[i] = createModule(moduleConfig);
            moduleTranslations[i] = moduleConfig.getPosition();
        }

        conversions = new SwerveConversions(config);
        kinematics = new SwerveDriveKinematics(moduleTranslations);
        speedsFactory = new SwerveSpeedsFactory(this);
        targetController = new SwerveTargetController(this);
        field = new SwerveField2d(this);

        double[] std = config.getStandardDeviations();
        poseEstimator =
            new SwerveDrivePoseEstimator(
                kinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(std[0], std[1], std[2]),
                VecBuilder.fill(0.0, 0.0, 0.0)
            );

        for (BlacklightConfig blacklightConfig : config.getBlacklights()) {
            Blacklight blacklight = new Blacklight(blacklightConfig);
            blacklight.publishConfig();
            blacklight.startListeners();
            blacklights.add(blacklight);
        }

        zeroIMU(0.0);

        System.out.println(
            "\nGRRSwerve Conversions:" +
            "\n\tModule Count: " +
            modules.length +
            "\n\tMove Rotations/Meter: " +
            Math2.toFixed(conversions.moveRotationsPerMeter()) +
            "\n\tTurn Rotations/Radian: " +
            Math2.toFixed(conversions.turnRotationsPerRadian())
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("velocityX", () -> Math2.toFixed(getVelocity(true).vxMetersPerSecond), null);
        builder.addDoubleProperty("velocityY", () -> Math2.toFixed(getVelocity(true).vyMetersPerSecond), null);
        builder.addDoubleProperty("velocityRot", () -> Math2.toFixed(getVelocity(true).omegaRadiansPerSecond), null);
        builder.addDoubleProperty(
            "velocityNorm",
            () -> {
                ChassisSpeeds velocity = getVelocity(true);
                return Math2.toFixed(Math.hypot(velocity.vxMetersPerSecond, velocity.vyMetersPerSecond));
            },
            null
        );

        builder.addDoubleProperty("odometryX", () -> Math2.toFixed(getPosition().getX()), null);
        builder.addDoubleProperty("odometryY", () -> Math2.toFixed(getPosition().getY()), null);
        builder.addDoubleProperty("odometryRot", () -> Math2.toFixed(getPosition().getRotation().getRadians()), null);

        for (SwerveModule module : modules) {
            GRRDashboard.addSubsystemSendable(
                "Modules/" + StringUtil.toPascalCase(module.getLabel()),
                this,
                SendableFactory.create(moduleBuilder -> {
                    moduleBuilder.publishConstString(".label", module.getLabel());
                    moduleBuilder.addDoubleProperty(
                        "velocity",
                        () -> Math2.toFixed(module.getVelocity()),
                        v -> module.setDesiredState(new SwerveModuleState(v, Rotation2d.fromRadians(module.getAngle())))
                    );
                    moduleBuilder.addDoubleProperty("distance", () -> Math2.toFixed(module.getDistance()), null);
                    moduleBuilder.addDoubleProperty(
                        "angle",
                        () -> Math2.toFixed(module.getAngle()),
                        v -> module.setDesiredState(new SwerveModuleState(0.0, Rotation2d.fromRadians(v)))
                    );
                })
            );
        }

        GRRDashboard.addSubsystemSendable("Field", this, field);
    }

    /**
     * Gets the IMU's yaw.
     */
    protected Rotation2d getYaw() {
        return Rotation2d.fromRadians(imu.getYaw());
    }

    /**
     * Gets the IMU's pitch.
     */
    protected Rotation2d getPitch() {
        return Rotation2d.fromRadians(imu.getPitch());
    }

    /**
     * Gets the IMU's roll.
     */
    protected Rotation2d getRoll() {
        return Rotation2d.fromRadians(imu.getRoll());
    }

    /**
     * Zeroes the IMU to the specified yaw and resets odometry to the same position facing the specified yaw.
     * @param yaw The yaw to zero to in radians.
     */
    protected void zeroIMU(double yaw) {
        imu.setZero(yaw);
        resetOdometry(new Pose2d(getPosition().getTranslation(), Rotation2d.fromRadians(yaw)));
    }

    /**
     * Gets the states of all swerve modules.
     */
    protected SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < moduleStates.length; i++) moduleStates[i] = modules[i].getState();
        return moduleStates;
    }

    /**
     * Gets the positions of all swerve modules.
     */
    protected SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modulePositions.length; i++) modulePositions[i] = modules[i].getPosition();
        return modulePositions;
    }

    /**
     * Gets the robot's velocity.
     * @param fieldRelative If the returned velocity should be field relative.
     */
    protected ChassisSpeeds getVelocity(boolean fieldRelative) {
        if (fieldRelative) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), getYaw().unaryMinus());
        } else {
            return kinematics.toChassisSpeeds(getModuleStates());
        }
    }

    /**
     * Gets the robot's position.
     */
    protected Pose2d getPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets odometry.
     * @param newPose The new pose.
     */
    protected void resetOdometry(Pose2d newPose) {
        poseEstimator.resetPosition(newPose.getRotation(), getModulePositions(), newPose);
        kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, newPose.getRotation()));
    }

    /**
     * Updates odometry.
     * Should be ran periodically.
     */
    protected void updateOdometry() {
        for (Blacklight blacklight : blacklights) blacklight.update(poseEstimator);
        poseEstimator.update(getYaw(), getModulePositions());
        field.update();
    }

    /**
     * Drives the modules into an X formation to prevent the robot from moving.
     */
    protected void lockWheels() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = new SwerveModuleState(0.0, moduleTranslations[i].getAngle());
        }

        targetController.setLastTarget(Math2.CHASSIS_SPEEDS_0, moduleStates);
        driveStates(moduleStates);
    }

    /**
     * Stops the modules.
     */
    protected void stop() {
        driveSpeeds(Math2.CHASSIS_SPEEDS_0, false, false);
    }

    /**
     * Drives using chassis speeds.
     * Speeds are discretized, then the target controller calculates module states.
     * @param chassisSpeeds The chassis speeds to drive with.
     */
    protected void driveSpeeds(ChassisSpeeds chassisSpeeds) {
        driveSpeeds(chassisSpeeds, true, true);
    }

    /**
     * Drives using chassis speeds.
     * @param chassisSpeeds The chassis speeds to drive with.
     * @param discretize If chassis speeds should be discretized.
     * @param withTargetController If the target controller should be used to calculate module states.
     */
    protected void driveSpeeds(ChassisSpeeds chassisSpeeds, boolean discretize, boolean withTargetController) {
        if (discretize) {
            double dtheta = chassisSpeeds.omegaRadiansPerSecond * config.getDiscretizationLookahead();
            double sin = -dtheta / 2.0;
            double cos = Math2.epsilonEquals(Math.cos(dtheta) - 1.0, 0.0)
                ? 1.0 - ((1.0 / 12.0) * dtheta * dtheta)
                : (sin * Math.sin(dtheta)) / (Math.cos(dtheta) - 1.0);

            double dt = config.getPeriod();
            double dx = chassisSpeeds.vxMetersPerSecond * dt;
            double dy = chassisSpeeds.vyMetersPerSecond * dt;
            chassisSpeeds =
                new ChassisSpeeds(((dx * cos) - (dy * sin)) / dt, ((dx * sin) + (dy * cos)) / dt, chassisSpeeds.omegaRadiansPerSecond);
        }

        if (withTargetController) {
            driveStates(targetController.calculate(chassisSpeeds).moduleStates());
        } else {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, config.getMaxV());
            targetController.setLastTarget(chassisSpeeds, states);
            driveStates(states);
        }
    }

    /**
     * Drives using raw swerve module states.
     * @param states The states to drive with.
     */
    protected void driveStates(SwerveModuleState[] states) {
        if (RobotBase.isSimulation()) {
            ((SwerveSimIMU) imu).updateAngle(kinematics.toChassisSpeeds(states));
        }

        for (int i = 0; i < states.length; i++) {
            SwerveModule module = modules[i];
            if (module != null) {
                module.setDesiredState(states[i]);
            }
        }
    }

    /**
     * Creates an IMU.
     */
    private SwerveIMU createIMU() {
        if (RobotBase.isSimulation()) return new SwerveSimIMU();

        Object[] imuArgs = config.getImuArgs();

        switch (config.getImuType()) {
            case ADIS16470:
                return new SwerveADIS16470(
                    createADIS16470(
                        "Swerve IMU",
                        (ADIS16470.IMUAxis) imuArgs[0],
                        (ADIS16470.IMUAxis) imuArgs[1],
                        (ADIS16470.IMUAxis) imuArgs[2],
                        (SPI.Port) imuArgs[3],
                        (ADIS16470.CalibrationTime) imuArgs[4]
                    )
                );
            case PIGEON2:
                return new SwervePigeon2(createPigeon2("Swerve IMU", (int) imuArgs[0], (String) imuArgs[1]), config);
            default:
                throw new UnsupportedOperationException("Invalid IMU type");
        }
    }

    /**
     * Creates a swerve module.
     * @param moduleConfig The module's config.
     */
    private SwerveModule createModule(SwerveModuleConfig moduleConfig) {
        SwerveAbsoluteEncoder absoluteEncoder = null;
        CANSparkMax turnSparkMax = null;

        switch (moduleConfig.getAbsoluteEncoderType()) {
            case CANCODER:
                absoluteEncoder =
                    new SwerveCANcoder(
                        createCANcoder(
                            moduleConfig.getLabel() + " Absolute Encoder",
                            moduleConfig.getAbsoluteEncoderDeviceId(),
                            moduleConfig.getAbsoluteEncoderCanBus()
                        ),
                        config,
                        moduleConfig
                    );
                break;
            case SPARK_MAX_ATTACHED:
                turnSparkMax =
                    createSparkMax(
                        moduleConfig.getLabel() + "Turn Motor",
                        moduleConfig.getTurnMotorDeviceId(),
                        config.getMoveMotorType().equals(SwerveMotorType.SPARK_MAX_BRUSHLESS) ? MotorType.kBrushless : MotorType.kBrushed
                    );

                absoluteEncoder =
                    new SwerveSparkMaxAttachedEncoder(
                        createSparkMaxAbsoluteEncoder(
                            moduleConfig.getLabel() + " Absolute Encoder",
                            turnSparkMax,
                            SparkMaxAbsoluteEncoder.Type.kDutyCycle
                        ),
                        config,
                        moduleConfig
                    );
                break;
            default:
                throw new UnsupportedOperationException("Invalid encoder type");
        }

        SwerveMotor moveMotor = createMotor(true, null, moduleConfig);
        SwerveMotor turnMotor = turnSparkMax != null
            ? new SwerveSparkMax(turnSparkMax, absoluteEncoder, config, moduleConfig)
            : createMotor(false, absoluteEncoder, moduleConfig);

        if (RobotBase.isSimulation()) {
            return new SwerveSimModule(moveMotor, turnMotor, absoluteEncoder, config, moduleConfig);
        } else {
            return new SwerveModule(moveMotor, turnMotor, absoluteEncoder, config, moduleConfig);
        }
    }

    /**
     * Creates a swerve motor.
     * @param isMoveMotor If the motor is a move motor.
     * @param absoluteEncoder If the motor is a turn motor, the absolute encoder. Otherwise {@code null}.
     * @param moduleConfig The module's config.
     */
    private SwerveMotor createMotor(boolean isMoveMotor, SwerveAbsoluteEncoder absoluteEncoder, SwerveModuleConfig moduleConfig) {
        String label = moduleConfig.getLabel() + " " + (isMoveMotor ? "Move" : "Turn") + " Motor";
        int deviceId = isMoveMotor ? moduleConfig.getMoveMotorDeviceId() : moduleConfig.getTurnMotorDeviceId();
        String canBus = isMoveMotor ? moduleConfig.getMoveMotorCanBus() : moduleConfig.getTurnMotorCanBus();

        switch (config.getMoveMotorType()) {
            case SPARK_MAX_BRUSHED:
            case SPARK_MAX_BRUSHLESS:
                return new SwerveSparkMax(
                    createSparkMax(
                        label,
                        deviceId,
                        config.getMoveMotorType().equals(SwerveMotorType.SPARK_MAX_BRUSHLESS) ? MotorType.kBrushless : MotorType.kBrushed
                    ),
                    absoluteEncoder,
                    config,
                    moduleConfig
                );
            case TALONFX:
                return new SwerveTalonFX(createTalonFX(label, deviceId, canBus), absoluteEncoder, config, moduleConfig);
            default:
                throw new UnsupportedOperationException("Invalid move motor type");
        }
    }
}
