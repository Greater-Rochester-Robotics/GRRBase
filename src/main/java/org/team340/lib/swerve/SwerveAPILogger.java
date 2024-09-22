package org.team340.lib.swerve;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.team340.lib.swerve.SwerveAPI.SwerveState;
import org.team340.lib.util.Profiler;

@CustomLoggerFor(SwerveAPI.class)
public class SwerveAPILogger extends ClassSpecificLogger<SwerveAPI> {

    public SwerveAPILogger() {
        super(SwerveAPI.class);
    }

    @Override
    public void update(DataLogger logger, SwerveAPI swerveAPI) {
        boolean profile = Profiler.isRunning();
        if (profile) Profiler.start("SwerveAPILogger");

        SwerveState state = swerveAPI.getState();
        logger.log("modulePositions", state.modulePositions, SwerveModulePosition.struct);
        logger.log("moduleStates", state.moduleStates, SwerveModuleState.struct);
        logger.log("targetStates", state.targetStates, SwerveModuleState.struct);
        logger.log("yaw", state.yaw, Rotation2d.struct);
        logger.log("pitch", state.roll, Rotation2d.struct);
        logger.log("roll", state.pitch, Rotation2d.struct);
        logger.log("pose", state.pose, Pose2d.struct);
        logger.log("speeds", state.speeds, ChassisSpeeds.struct);

        ErrorHandler errorHandler = Epilogue.getConfig().errorHandler;
        swerveAPI.imu.log(logger.getSubLogger("imu"), errorHandler);
        for (var module : swerveAPI.modules) {
            DataLogger moduleLogger = logger.getSubLogger(module.getName());
            module.moveMotor.log(moduleLogger.getSubLogger("moveMotor"), errorHandler);
            module.turnMotor.log(moduleLogger.getSubLogger("turnMotor"), errorHandler);
            module.encoder.log(moduleLogger.getSubLogger("encoder"), errorHandler);
        }

        if (profile) Profiler.end();
    }
}
