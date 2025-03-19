package org.team340.robot.subsystems;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.team340.lib.swerve.Perspective;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveState;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double kMoveRatio = (54.0 / 10.0) * (18.0 / 38.0) * (45.0 / 15.0);
    private static final double kTurnRatio = (22.0 / 10.0) * (88.0 / 16.0);
    private static final double kModuleOffset = Units.inchesToMeters(12.5);

    private static final SwerveModuleConfig kFrontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(kModuleOffset, kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kFlMove, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kFlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kFlEncoder, 0.0, false));

    private static final SwerveModuleConfig kFrontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(kModuleOffset, -kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kFrMove, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kFrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kFrEncoder, 0.0, false));

    private static final SwerveModuleConfig kBackLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-kModuleOffset, kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kBlMove, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kBlTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kBlEncoder, 0.0, false));

    private static final SwerveModuleConfig kBackRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-kModuleOffset, -kModuleOffset)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.kBrMove, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.kBrTurn, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.kBrEncoder, 0.0, false));

    private static final SwerveConfig kConfig = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, 0.004, 0.02, 0.01)
        .setMovePID(0.25, 0.0, 0.0)
        .setMoveFF(0.0, 0.125)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(false, true)
        .setLimits(4.5, 0.05, 17.5, 14.0, 30.0)
        .setDriverProfile(4.0, 1.5, 0.15, 4.75, 2.0, 0.05)
        .setPowerProperties(Constants.kVoltage, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(kMoveRatio, kTurnRatio, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.canandgyro(RobotMap.kCanandgyro))
        .setPhoenixFeatures(new CANBus(RobotMap.kLowerCANBus), true, true, true)
        .setModules(kFrontLeft, kFrontRight, kBackLeft, kBackRight);

    private final SwerveAPI api;
    private final SwerveState state;

    private final PIDController autoPIDx;
    private final PIDController autoPIDy;
    private final PIDController autoPIDangular;

    @SuppressWarnings("unused")
    private Pose2d autoLast = null;

    private Pose2d autoNext = null;

    public Swerve() {
        api = new SwerveAPI(kConfig);
        state = api.state;

        autoPIDx = new PIDController(10.0, 0.0, 0.0);
        autoPIDy = new PIDController(10.0, 0.0, 0.0);
        autoPIDangular = new PIDController(10.0, 0.0, 0.0);
        autoPIDangular.enableContinuousInput(-Math.PI, Math.PI);

        api.enableTunables("swerve/api");
        Tunable.pidController("swerve/autoPID", autoPIDx);
        Tunable.pidController("swerve/autoPID", autoPIDy);
        Tunable.pidController("swerve/autoPIDangular", autoPIDangular);
    }

    @Override
    public void periodic() {
        api.refresh();
    }

    /**
     * Returns the current blue origin relative pose of the robot.
     */
    @NotLogged
    public Pose2d getPose() {
        return state.pose;
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(Perspective.kOperator))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(
                x.getAsDouble(),
                y.getAsDouble(),
                angular.getAsDouble(),
                Perspective.kOperator,
                true,
                true
            )
        );
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement. This
     * method is not intended for use outside of creating an {@link AutoFactory}.
     * @param pose The new blue origin relative pose to apply to the pose estimator.
     */
    public void resetPose(Pose2d pose) {
        api.resetPose(pose);

        autoPIDx.reset();
        autoPIDy.reset();
        autoPIDangular.reset();
    }

    /**
     * Follows a Choreo trajectory by moving towards the next sample. This method
     * is not intended for use outside of creating an {@link AutoFactory}.
     * @param sample The next trajectory sample.
     */
    public void followTrajectory(SwerveSample sample) {
        autoLast = autoNext;
        autoNext = sample.getPose();

        Pose2d pose = state.pose;
        api.applySpeeds(
            new ChassisSpeeds(
                sample.vx + autoPIDx.calculate(pose.getX(), sample.x),
                sample.vy + autoPIDy.calculate(pose.getY(), sample.y),
                sample.omega + autoPIDangular.calculate(pose.getRotation().getRadians(), sample.heading)
            ),
            Perspective.kBlue,
            true,
            false
        );
    }
}
