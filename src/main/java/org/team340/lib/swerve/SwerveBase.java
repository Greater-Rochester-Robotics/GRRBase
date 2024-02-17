package org.team340.lib.swerve;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.Consumer;
import org.team340.lib.GRRDashboard;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.encoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.encoders.vendors.SwerveCANcoder;
import org.team340.lib.swerve.hardware.encoders.vendors.SwerveSparkEncoder;
import org.team340.lib.swerve.hardware.imu.SwerveIMU;
import org.team340.lib.swerve.hardware.imu.SwerveIMUSim;
import org.team340.lib.swerve.hardware.imu.vendors.SwerveADIS16470;
import org.team340.lib.swerve.hardware.imu.vendors.SwervePigeon2;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.swerve.hardware.motors.vendors.SwerveSparkFlex;
import org.team340.lib.swerve.hardware.motors.vendors.SwerveSparkMax;
import org.team340.lib.swerve.hardware.motors.vendors.SwerveTalonFX;
import org.team340.lib.swerve.util.SwerveConversions;
import org.team340.lib.swerve.util.SwerveField2d;
import org.team340.lib.swerve.util.SwerveOdometryThread;
import org.team340.lib.swerve.util.SwerveRatelimiter;
import org.team340.lib.swerve.util.SwerveSerializer;
import org.team340.lib.util.Math2;
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
 *   <li>Brushed Spark Flex</li>
 *   <li>Brushless Spark Flex</li>
 *   <li>Talon FX</li>
 * </ul>
 *
 * Supported encoders:
 * <ul>
 *   <li>CANcoder</li>
 *   <li>Spark Attached Encoders</li>
 * </ul>
 *
 * Supported IMUs:
 * <ul>
 *   <li>ADIS16470</li>
 *   <li>Pigeon 2</li>
 * </ul>
 */
public abstract class SwerveBase extends GRRSubsystem {

    protected final SwerveConfig config;
    protected final SwerveConversions conversions;
    protected final SwerveIMU imu;
    protected final SwerveDriveKinematics kinematics;
    protected final SwerveModule[] modules;
    protected final SwerveDrivePoseEstimator poseEstimator;
    protected final SysIdRoutine sysIdRoutine;

    private final Translation2d[] moduleTranslations;
    private final SwerveOdometryThread odometryThread;
    private final SwerveRatelimiter ratelimiter;
    private final SwerveField2d field;

    private final MutableMeasure<Voltage> sysIdAppliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> sysIdDistance = mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> sysIdVelocity = mutable(MetersPerSecond.of(0));

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
        odometryThread = new SwerveOdometryThread(modules, imu::getYaw, config);
        ratelimiter = new SwerveRatelimiter(config, kinematics, getModuleStates());
        field = new SwerveField2d(config);

        double[] std = config.getOdometryStd();
        poseEstimator =
            new SwerveDrivePoseEstimator(
                kinematics,
                imu.getYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(std[0], std[1], std[2]),
                VecBuilder.fill(0.0, 0.0, 0.0)
            );

        sysIdRoutine =
            new SysIdRoutine(
                config.getSysIdConfig(),
                new Mechanism(
                    (Measure<Voltage> volts) -> {
                        driveVoltage(volts.in(Volts), Math2.ROTATION2D_0);
                    },
                    log -> {
                        for (SwerveModule module : modules) {
                            log
                                .motor("module-" + StringUtil.toCamelCase(module.getLabel()))
                                .voltage(
                                    sysIdAppliedVoltage.mut_replace(module.getMoveDutyCycle() * RobotController.getBatteryVoltage(), Volts)
                                )
                                .linearPosition(sysIdDistance.mut_replace(module.getDistance(), Meters))
                                .linearVelocity(sysIdVelocity.mut_replace(module.getVelocity(), MetersPerSecond));
                        }
                    },
                    this,
                    "Swerve"
                )
            );

        imu.setZero(Math2.ROTATION2D_0);
        odometryThread.start();

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

        builder.addDoubleProperty("velocityX", () -> getVelocity(true).vxMetersPerSecond, null);
        builder.addDoubleProperty("velocityY", () -> getVelocity(true).vyMetersPerSecond, null);
        builder.addDoubleProperty("velocityRot", () -> getVelocity(true).omegaRadiansPerSecond, null);
        builder.addDoubleProperty("velocityNorm", () -> SwerveSerializer.chassisSpeedsNorm(getVelocity(true)), null);

        builder.addDoubleProperty("odometryX", () -> getPosition().getX(), null);
        builder.addDoubleProperty("odometryY", () -> getPosition().getY(), null);
        builder.addDoubleProperty("odometryRot", () -> getPosition().getRotation().getRadians(), null);

        builder.addDoubleArrayProperty(
            "desiredModuleStates",
            () -> SwerveSerializer.moduleStatesDoubleArray(getDesiredModuleStates()),
            null
        );
        builder.addDoubleArrayProperty("moduleStates", () -> SwerveSerializer.moduleStatesDoubleArray(getModuleStates()), null);

        for (SwerveModule module : modules) {
            GRRDashboard.addSubsystemSendable(
                "Modules/" + StringUtil.toPascalCase(module.getLabel()),
                this,
                SendableFactory.create(moduleBuilder -> {
                    moduleBuilder.publishConstString(".label", module.getLabel());
                    moduleBuilder.addDoubleProperty("velocity", module::getVelocity, null);
                    moduleBuilder.addDoubleProperty("distance", module::getDistance, null);
                    moduleBuilder.addDoubleProperty("angle", module::getHeading, null);
                })
            );
        }

        GRRDashboard.addSubsystemSendable("Field", this, field);
    }

    /**
     * Gets the robot's velocity.
     * @param fieldRelative If the returned velocity should be field relative.
     */
    protected ChassisSpeeds getVelocity(boolean fieldRelative) {
        if (fieldRelative) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getModuleStates()), imu.getYaw().unaryMinus());
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
     * Gets the desired states of all swerve modules.
     */
    protected SwerveModuleState[] getDesiredModuleStates() {
        SwerveModuleState[] desiredModuleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < desiredModuleStates.length; i++) desiredModuleStates[i] = modules[i].getDesiredState();
        return desiredModuleStates;
    }

    /**
     * Gets the states of all swerve modules.
     */
    protected SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < moduleStates.length; i++) moduleStates[i] = modules[i].getModuleState();
        return moduleStates;
    }

    /**
     * Gets the positions of all swerve modules.
     */
    protected SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modulePositions.length; i++) modulePositions[i] = modules[i].getModulePosition();
        return modulePositions;
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
        updateOdometry(poseEstimator -> {});
    }

    /**
     * Updates odometry.
     * Should be ran periodically.
     * @param poseEstimatorConsumer A consumer that accepts the pose estimator. Should be used for applying field-relative poses from vision data. Note that it is expected that standard deviations are specified when using {@link SwerveDrivePoseEstimator#addVisionMeasurement}, as the initial standard deviations are set to {@code 0.0}.
     */
    protected void updateOdometry(Consumer<SwerveDrivePoseEstimator> poseEstimatorConsumer) {
        odometryThread.update(poseEstimator);
        poseEstimatorConsumer.accept(poseEstimator);
        field.update(getPosition());
    }

    /**
     * Drives the modules into an X formation to prevent the robot from moving.
     */
    protected void lockWheels() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < moduleStates.length; i++) {
            moduleStates[i] = new SwerveModuleState(0.0, moduleTranslations[i].getAngle());
        }

        ratelimiter.setLastState(Math2.CHASSIS_SPEEDS_0, moduleStates);
        driveStates(moduleStates);
    }

    /**
     * Stops the modules.
     */
    protected void stop() {
        driveSpeeds(Math2.CHASSIS_SPEEDS_0, false, false);
    }

    /**
     * Drives the robot using percents of its calculated max velocity.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     * @param rot The desired rotational speed from {@code -1.0} to {@code 1.0}.
     * @param fieldRelative If the robot should drive field relative.
     */
    protected void drive(double x, double y, double rot, boolean fieldRelative) {
        driveVelocity(x * config.getVelocity(), y * config.getVelocity(), rot * config.getRotationalVelocity(), fieldRelative);
    }

    /**
     * Drives the robot using velocity.
     * @param xV The desired {@code x} velocity in meters/second.
     * @param yV The desired {@code y} velocity in meters/second.
     * @param rotV The desired rotational velocity in radians/second.
     * @param fieldRelative If the robot should drive field relative.
     */
    protected void driveVelocity(double xV, double yV, double rotV, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xV, yV, rotV, imu.getYaw())
            : new ChassisSpeeds(xV, yV, rotV);

        driveSpeeds(chassisSpeeds);
    }

    /**
     * Drives the robot using percents of its calculated max velocity while locked pointing at a position on the field.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     * @param angleOffset The offset to use when determining which side of the robot should face the point. (value in radians)
     * @param point The desired field relative position to point at (axis values in meters).
     * @param controller A profiled PID controller to use for translating to and maintaining the angle to the desired point.
     */
    protected void driveAroundPoint(double x, double y, double angleOffset, Translation2d point, ProfiledPIDController controller) {
        driveAroundPointVelocity(x * config.getVelocity(), y * config.getVelocity(), angleOffset, point, controller);
    }

    /**
     * Drives the robot using velocity while locked pointing at a position on the field.
     * @param xV The desired {@code x} velocity in meters/second.
     * @param yV The desired {@code y} velocity in meters/second.
     * @param angleOffset The offset to use when determining which side of the robot should face the point. (value in radians)
     * @param point The desired field relative position to point at (axis values in meters).
     * @param controller A profiled PID controller to use for translating to and maintaining the angle to the desired point.
     */
    protected void driveAroundPointVelocity(
        double xV,
        double yV,
        double angleOffset,
        Translation2d point,
        ProfiledPIDController controller
    ) {
        Translation2d robotPoint = getPosition().getTranslation();
        double angle = MathUtil.angleModulus(point.minus(robotPoint).getAngle().getRadians() + angleOffset);
        driveAngleVelocity(xV, yV, angle, controller);
    }

    /**
     * Drives the robot using percents of its calculated max velocity while locked at a field relative angle.
     * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
     * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
     * @param angle The desired field relative angle to point at in radians.
     * @param controller A profiled PID controller to use for translating to and maintaining the angle.
     */
    protected void driveAngle(double x, double y, double angle, ProfiledPIDController controller) {
        driveAngleVelocity(x * config.getVelocity(), y * config.getVelocity(), angle, controller);
    }

    /**
     * Drives the robot using velocity while locked at a field relative angle.
     * @param xV The desired {@code x} velocity in meters/second.
     * @param yV The desired {@code y} velocity in meters/second.
     * @param angle The desired field relative angle to point at in radians.
     * @param controller A profiled PID controller to use for translating to and maintaining the angle.
     */
    protected void driveAngleVelocity(double xV, double yV, double angle, ProfiledPIDController controller) {
        Rotation2d yaw = imu.getYaw();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xV,
            yV,
            controller.calculate(MathUtil.angleModulus(yaw.getRadians()), angle),
            yaw
        );

        driveSpeeds(chassisSpeeds);
    }

    /**
     * Drives the robot to a field relative pose.
     * @param pose The pose to drive to.
     * @param xController A PID controller to use for translating to and maintaining pose's {@code x} position.
     * @param yController The PID controller to use for translating to and maintaining pose's {@code y} position.
     * @param rotController The profiled PID controller to use for translating to and maintaining the pose's angle.
     */
    protected void driveToPose(Pose2d pose, PIDController xController, PIDController yController, ProfiledPIDController rotController) {
        Rotation2d yaw = imu.getYaw();
        Pose2d position = getPosition();
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(position.getX(), pose.getX()),
            yController.calculate(position.getY(), pose.getY()),
            rotController.calculate(MathUtil.angleModulus(yaw.getRadians()), pose.getRotation().getRadians()),
            yaw
        );

        driveSpeeds(chassisSpeeds);
    }

    /**
     * Drives using chassis speeds.
     * Speeds are discretized, then the ratelimiter calculates module states.
     * @param chassisSpeeds The chassis speeds to drive with.
     */
    protected void driveSpeeds(ChassisSpeeds chassisSpeeds) {
        driveSpeeds(chassisSpeeds, true, true);
    }

    /**
     * Drives using chassis speeds.
     * @param chassisSpeeds The chassis speeds to drive with.
     * @param discretize If chassis speeds should be discretized.
     * @param withRatelimiter If the ratelimiter should be used to calculate module states.
     */
    protected void driveSpeeds(ChassisSpeeds chassisSpeeds, boolean discretize, boolean withRatelimiter) {
        if (discretize) {
            // Calculate how much the robot will have turned in configured lookahead number of seconds.
            double dtheta = chassisSpeeds.omegaRadiansPerSecond * config.getDiscretizationLookahead();

            // Find the coefficients of the twist experienced by the robot.
            double sin = -dtheta / 2.0;
            double cos = Math2.epsilonEquals(Math.cos(dtheta) - 1.0, 0.0)
                ? 1.0 - ((1.0 / 12.0) * dtheta * dtheta)
                : (sin * Math.sin(dtheta)) / (Math.cos(dtheta) - 1.0);

            // Find distance traveled over lookahead period.
            double dt = config.getPeriod();
            double dx = chassisSpeeds.vxMetersPerSecond * dt;
            double dy = chassisSpeeds.vyMetersPerSecond * dt;

            // Apply the found twist to the chassis speeds.
            chassisSpeeds =
                new ChassisSpeeds(((dx * cos) - (dy * sin)) / dt, ((dx * sin) + (dy * cos)) / dt, chassisSpeeds.omegaRadiansPerSecond);
        }

        if (withRatelimiter) {
            driveStates(ratelimiter.calculate(chassisSpeeds).moduleStates());
        } else {
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, config.getVelocity());
            ratelimiter.setLastState(chassisSpeeds, states);
            driveStates(states);
        }
    }

    /**
     * Drives the robot's move motors at a specified voltage.
     * Useful for feedforward characterization.
     * @param voltage The voltage to apply to the move motors.
     * @param heading A robot relative heading for the modules to point towards.
     */
    protected void driveVoltage(double voltage, Rotation2d heading) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setVoltage(voltage, heading);
        }
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(0, heading);
        }
        ratelimiter.setLastState(new SwerveRatelimiter.SwerveState(new ChassisSpeeds(), states));
    }

    /**
     * Drives using raw swerve module states.
     * @param states The states to drive with.
     */
    protected void driveStates(SwerveModuleState[] states) {
        if (RobotBase.isSimulation()) {
            ((SwerveIMUSim) imu).updateSim(kinematics.toChassisSpeeds(states));
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
        if (RobotBase.isSimulation()) return new SwerveIMUSim();

        Object[] imuArgs = config.getImuArgs();

        switch (config.getImuType()) {
            case ADIS16470:
                return new SwerveADIS16470(
                    createADIS16470(
                        "Swerve IMU",
                        (ADIS16470_IMU.IMUAxis) imuArgs[0],
                        (ADIS16470_IMU.IMUAxis) imuArgs[1],
                        (ADIS16470_IMU.IMUAxis) imuArgs[2],
                        (SPI.Port) imuArgs[3],
                        (ADIS16470_IMU.CalibrationTime) imuArgs[4]
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
        SwerveEncoder encoder = null;
        CANSparkMax turnSparkMax = null;
        CANSparkFlex turnSparkFlex = null;

        switch (moduleConfig.getEncoderType()) {
            case CANCODER:
                encoder =
                    new SwerveCANcoder(
                        createCANcoder(
                            moduleConfig.getLabel() + " Absolute Encoder",
                            moduleConfig.getEncoderDeviceId(),
                            moduleConfig.getEncoderCanBus()
                        ),
                        config,
                        moduleConfig
                    );
                break;
            case SPARK_ENCODER:
                if (
                    config.getMoveMotorType().equals(SwerveMotor.Type.SPARK_MAX_BRUSHED) ||
                    config.getMoveMotorType().equals(SwerveMotor.Type.SPARK_MAX_BRUSHLESS)
                ) {
                    turnSparkMax =
                        createSparkMax(
                            moduleConfig.getLabel() + "Turn Motor",
                            moduleConfig.getTurnMotorDeviceId(),
                            config.getMoveMotorType().equals(SwerveMotor.Type.SPARK_MAX_BRUSHLESS)
                                ? MotorType.kBrushless
                                : MotorType.kBrushed
                        );

                    encoder =
                        new SwerveSparkEncoder(
                            createSparkMaxAbsoluteEncoder(
                                moduleConfig.getLabel() + " Absolute Encoder",
                                turnSparkMax,
                                SparkAbsoluteEncoder.Type.kDutyCycle
                            )
                        );
                } else {
                    turnSparkFlex =
                        createSparkFlex(
                            moduleConfig.getLabel() + "Turn Motor",
                            moduleConfig.getTurnMotorDeviceId(),
                            config.getMoveMotorType().equals(SwerveMotor.Type.SPARK_FLEX_BRUSHLESS)
                                ? MotorType.kBrushless
                                : MotorType.kBrushed
                        );

                    encoder =
                        new SwerveSparkEncoder(
                            createSparkFlexAbsoluteEncoder(
                                moduleConfig.getLabel() + " Absolute Encoder",
                                turnSparkFlex,
                                SparkAbsoluteEncoder.Type.kDutyCycle
                            )
                        );
                }
                break;
            default:
                throw new UnsupportedOperationException("Invalid encoder type");
        }

        SwerveMotor moveMotor = createMotor(true, null, moduleConfig);
        SwerveMotor turnMotor = turnSparkMax != null
            ? new SwerveSparkMax(false, turnSparkMax, encoder, config, moduleConfig)
            : (
                turnSparkFlex != null
                    ? new SwerveSparkFlex(false, turnSparkFlex, encoder, config, moduleConfig)
                    : createMotor(false, encoder, moduleConfig)
            );

        return new SwerveModule(moveMotor, turnMotor, encoder, config, moduleConfig);
    }

    /**
     * Creates a swerve motor.
     * @param isMoveMotor If the motor is a move motor.
     * @param encoder If the motor is a turn motor, the absolute encoder. Otherwise {@code null}.
     * @param moduleConfig The module's config.
     */
    private SwerveMotor createMotor(boolean isMoveMotor, SwerveEncoder encoder, SwerveModuleConfig moduleConfig) {
        String label = moduleConfig.getLabel() + " " + (isMoveMotor ? "Move" : "Turn") + " Motor";
        int deviceId = isMoveMotor ? moduleConfig.getMoveMotorDeviceId() : moduleConfig.getTurnMotorDeviceId();
        String canBus = isMoveMotor ? moduleConfig.getMoveMotorCanBus() : moduleConfig.getTurnMotorCanBus();

        switch (config.getMoveMotorType()) {
            case SPARK_MAX_BRUSHED:
            case SPARK_MAX_BRUSHLESS:
                return new SwerveSparkMax(
                    isMoveMotor,
                    createSparkMax(
                        label,
                        deviceId,
                        config.getMoveMotorType().equals(SwerveMotor.Type.SPARK_MAX_BRUSHLESS) ? MotorType.kBrushless : MotorType.kBrushed
                    ),
                    encoder,
                    config,
                    moduleConfig
                );
            case SPARK_FLEX_BRUSHED:
            case SPARK_FLEX_BRUSHLESS:
                return new SwerveSparkFlex(
                    isMoveMotor,
                    createSparkFlex(
                        label,
                        deviceId,
                        config.getMoveMotorType().equals(SwerveMotor.Type.SPARK_FLEX_BRUSHLESS) ? MotorType.kBrushless : MotorType.kBrushed
                    ),
                    encoder,
                    config,
                    moduleConfig
                );
            case TALONFX:
                return new SwerveTalonFX(isMoveMotor, createTalonFX(label, deviceId, canBus), config, moduleConfig);
            default:
                throw new UnsupportedOperationException("Invalid move motor type");
        }
    }
}
