package org.team340.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import org.team340.lib.swerve.SwerveAPI;
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
        .setTimings(Constants.PERIOD, 0.005, 0.025)
        .setFieldSize(Constants.FIELD_LENGTH, Constants.FIELD_WIDTH)
        .setMovePID(0.1, 0.0, 0.0, 0.0)
        .setMoveFF(0.0, 2.4, 0.0)
        .setTurnPID(0.5, 0.0, 0.0, 0.0)
        .setBrakeMode(false, true)
        .setMaxSpeeds(5.0, 10.0)
        .setRatelimits(15.0, 30.0)
        .setPowerProperties(Constants.VOLTAGE, 80.0, 40.0)
        .setMechanicalProperties(75.0 / 14.0, 18.75, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setVisionStd(0.0, 0.0, 0.0)
        .setIMU(SwerveIMUs.pigeon2(RobotMap.PIGEON))
        .setPhoenixFeatures(RobotMap.CANBUS, true, true, true)
        .setModules(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

    private final SwerveAPI api;

    public Swerve() {
        api = new SwerveAPI(CONFIG);
    }

    @Override
    public void periodic() {
        api.refresh();
    }
}
