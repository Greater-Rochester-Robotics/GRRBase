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

@CustomLoggerFor(SwerveAPI.class)
public class SwerveAPILogger extends ClassSpecificLogger<SwerveAPI> {

    public SwerveAPILogger() {
        super(SwerveAPI.class);
    }

    @Override
    public void update(DataLogger logger, SwerveAPI swerveAPI) {
        logState(logger.getSubLogger("state"), swerveAPI.getState());
        var hardwareLogger = logger.getSubLogger("hardware");

        ErrorHandler errorHandler = Epilogue.getConfig().errorHandler;
        swerveAPI.imu.log(hardwareLogger.getSubLogger("imu"), errorHandler);
        for (var module : swerveAPI.modules) {
            var moduleLogger = hardwareLogger.getSubLogger(module.getName());
            module.moveMotor.log(moduleLogger.getSubLogger("moveMotor"), errorHandler);
            module.turnMotor.log(moduleLogger.getSubLogger("turnMotor"), errorHandler);
            module.encoder.log(moduleLogger.getSubLogger("encoder"), errorHandler);
        }
    }

    private void logState(DataLogger logger, SwerveState state) {
        logger.log("pitch", state.pitch, Rotation2d.struct);
        logger.log("roll", state.roll, Rotation2d.struct);
        logger.log("pose", state.pose, Pose2d.struct);
        logger.log("speeds", state.speeds, ChassisSpeeds.struct);
        logger.log("targetSpeeds", state.targetSpeeds, ChassisSpeeds.struct);
        logger.log("velocity", state.velocity);

        var modules = logger.getSubLogger("modules");
        modules.log("positions", state.modules.positions, SwerveModulePosition.struct);
        modules.log("states", state.modules.states, SwerveModuleState.struct);
        modules.log("nextTarget", state.modules.nextTarget, SwerveModuleState.struct);
        modules.log("lastTarget", state.modules.lastTarget, SwerveModuleState.struct);

        var odometry = logger.getSubLogger("odometry");
        odometry.log("timesync", state.odometry.timesync);
        odometry.log("successes", state.odometry.successes);
        odometry.log("failures", state.odometry.failures);
    }
}
