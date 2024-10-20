package org.team340.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveAPI.ForwardPerspective;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

@Logged
public class Swerve extends GRRSubsystem {

    private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(0.28, 0.28)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FL_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FL_TURN, false))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.FL_ENCODER, 0.0, false));

    private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(0.28, -0.28)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.FR_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.FR_TURN, false))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.FR_ENCODER, 0.0, false));

    private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-0.28, 0.28)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BL_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BL_TURN, false))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.BL_ENCODER, 0.0, false));

    private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-0.28, -0.28)
        .setMoveMotor(SwerveMotors.talonFX(RobotMap.BR_MOVE, false))
        .setTurnMotor(SwerveMotors.talonFX(RobotMap.BR_TURN, false))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.BR_ENCODER, 0.0, false));

    private static final SwerveConfig CONFIG = new SwerveConfig()
        .setTimings(Constants.PERIOD, 0.004, 0.02)
        .setMovePID(0.1, 0.0, 0.0, 0.0)
        .setMoveFF(0.0, 2.4)
        .setTurnPID(0.5, 0.0, 0.0, 0.0)
        .setBrakeMode(false, true)
        .setLimits(5.0, 16.0, 12.0, 31.416)
        .setDriverProfile(4.5, 1.0, 5.498, 2.0)
        .setPowerProperties(12.0, 80.0, 60.0)
        .setMechanicalProperties(75.0 / 14.0, 18.75, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.pigeon2(RobotMap.PIGEON))
        .setPhoenixFeatures(new CANBus(RobotMap.CANBUS), true, true, true)
        .setModules(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

    private final SwerveAPI api;

    public Swerve() {
        api = new SwerveAPI(CONFIG);
        api.enableTunables("Swerve");
    }

    @Override
    public void periodic() {
        api.refresh();
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(x.get(), y.get(), angular.get(), ForwardPerspective.OPERATOR, true, true)
        );
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(ForwardPerspective.OPERATOR))
            .isFinished(true)
            .ignoringDisable(true);
    }
}
