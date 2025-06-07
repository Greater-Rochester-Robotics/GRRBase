package org.team340.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.team340.lib.swerve.Perspective;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveState;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.Math2;
import org.team340.lib.util.PAPFController;
import org.team340.lib.util.PAPFController.Obstacle;
import org.team340.lib.util.Tunable;
import org.team340.lib.util.command.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

/**
 * The robot's swerve drivetrain.
 */
@Logged
public final class Swerve extends GRRSubsystem {

    private static final double OFFSET = Units.inchesToMeters(12.5);

    private final SwerveModuleConfig frontLeft = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FL_ENCODER, 0.0, false));

    private final SwerveModuleConfig frontRight = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.FR_ENCODER, 0.0, false));

    private final SwerveModuleConfig backLeft = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-OFFSET, OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BL_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BL_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BL_ENCODER, 0.0, false));

    private final SwerveModuleConfig backRight = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-OFFSET, -OFFSET)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BR_MOVE, true))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BR_TURN, true))
        .setEncoder(SwerveEncoders.cancoder(RobotMap.BR_ENCODER, 0.0, false));

    private final SwerveConfig config = new SwerveConfig()
        .setTimings(TimedRobot.kDefaultPeriod, 0.004, 0.02, 0.01)
        .setMovePID(0.25, 0.0, 0.0)
        .setMoveFF(0.0, 0.125)
        .setTurnPID(100.0, 0.0, 0.2)
        .setBrakeMode(false, true)
        .setLimits(4.5, 0.05, 17.5, 14.0, 30.0)
        .setDriverProfile(4.0, 1.5, 0.15, 4.75, 2.0, 0.05)
        .setPowerProperties(Constants.VOLTAGE, 100.0, 80.0, 60.0, 60.0)
        .setMechanicalProperties(729.0 / 95.0, 12.1, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.canandgyro(RobotMap.CANANDGYRO))
        .setPhoenixFeatures(new CANBus(RobotMap.LOWER_CAN), true, true, true)
        .setModules(frontLeft, frontRight, backLeft, backRight);

    private final SwerveAPI api;
    private final SwerveState state;

    private final PAPFController apf;
    private final ProfiledPIDController angularPID;

    public Swerve() {
        api = new SwerveAPI(config);
        state = api.state;

        apf = new PAPFController(4.0, 0.5, 0.01, true, new Obstacle[0]);
        angularPID = new ProfiledPIDController(10.0, 0.0, 0.0, new Constraints(10.0, 25.0));
        angularPID.enableContinuousInput(-Math.PI, Math.PI);

        api.enableTunables("swerve/api");
        apf.enableTunables("swerve/apf");
        Tunable.pidController("swerve/angularPID", angularPID);
    }

    @Override
    public void periodic() {
        api.refresh();
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(Perspective.OPERATOR))
            .isFinished(true)
            .ignoringDisable(true);
    }

    /**
     * Resets the pose of the robot, inherently seeding field-relative movement.
     * @param pose A supplier that returns the new blue origin relative pose to apply to the pose estimator.
     */
    public Command resetPose(Supplier<Pose2d> pose) {
        return commandBuilder("Swerve.resetPose()")
            .onInitialize(() -> api.resetPose(pose.get()))
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
                Perspective.OPERATOR,
                true,
                true
            )
        );
    }

    /**
     * Drives the robot to a target position using the P-APF, until the
     * robot is positioned within a specified tolerance of the target.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     * @param endTolerance The tolerance in meters at which to end the command.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration, DoubleSupplier endTolerance) {
        return apfDrive(goal, maxDeceleration)
            .until(() -> Math2.isNear(goal.get().getTranslation(), state.translation, endTolerance.getAsDouble()))
            .withName("Swerve.apfDrive()");
    }

    /**
     * Drives the robot to a target position using the P-APF. This command does not end.
     * @param goal A supplier that returns the target blue-origin relative field location.
     * @param maxDeceleration A supplier that returns the desired deceleration rate of the robot, in m/s/s.
     */
    public Command apfDrive(Supplier<Pose2d> goal, DoubleSupplier maxDeceleration) {
        return commandBuilder("Swerve.apfDrive()")
            .onInitialize(() -> angularPID.reset(state.rotation.getRadians(), state.speeds.omegaRadiansPerSecond))
            .onExecute(() -> {
                Pose2d next = goal.get();
                var speeds = apf.calculate(
                    state.pose,
                    next.getTranslation(),
                    config.velocity,
                    maxDeceleration.getAsDouble()
                );

                speeds.omegaRadiansPerSecond = angularPID.calculate(
                    state.rotation.getRadians(),
                    next.getRotation().getRadians()
                );

                api.applySpeeds(speeds, Perspective.BLUE, true, true);
            });
    }

    /**
     * Drives the modules to stop the robot from moving.
     * @param lock If the wheels should be driven to an X formation to stop the robot from being pushed.
     */
    public Command stop(boolean lock) {
        return commandBuilder("Swerve.stop(" + lock + ")").onExecute(() -> api.applyStop(lock));
    }
}
