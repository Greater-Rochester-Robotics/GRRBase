package org.team340.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.team340.lib.swerve.config.SwerveConfig;

/**
 * A controller that applies constraints onto the speeds applied to swerve modules to account for physical limitations of the robot.
 */
public class SwerveRatelimiter {

    /**
     * A state.
     */
    public static record SwerveState(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {}

    /**
     * The general config.
     */
    private final SwerveConfig config;
    /**
     * The kinematics instance in use.
     */
    private final SwerveDriveKinematics kinematics;

    /**
     * The last swerve state.
     */
    private SwerveState lastState;

    /**
     * Create the ratelimiter.
     * @param config The general swerve config.
     * @param kinematics The kinematics instance used by the swerve subsystem.
     * @param initialModuleStates The initial states of the swerve modules.
     */
    public SwerveRatelimiter(SwerveConfig config, SwerveDriveKinematics kinematics, SwerveModuleState[] initialModuleStates) {
        this.config = config;
        this.kinematics = kinematics;
        this.lastState = new SwerveState(kinematics.toChassisSpeeds(initialModuleStates), initialModuleStates);
    }

    /**
     * Gets the ratelimiter's last state.
     */
    public SwerveState getLastState() {
        return lastState;
    }

    /**
     * Sets the ratelimiter's last state.
     * @param chassisSpeeds The speeds for the state.
     * @param moduleStates The module states to use.
     */
    public void setLastState(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
        setLastState(new SwerveState(chassisSpeeds, moduleStates));
    }

    /**
     * Sets the ratelimiter's last state.
     * @param state The state to use.
     */
    public void setLastState(SwerveState state) {
        lastState = state;
    }

    /**
     * Calculates a new state.
     * @param desiredSpeeds The new desired chassis speeds.
     * @return The new state.
     */
    public SwerveState calculate(ChassisSpeeds desiredSpeeds) {
        // - Determine the desired swerve module states.
        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);

        // - Preliminary wheel speed desaturation based on the configured max robot velocity.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, config.getMaxV());

        // - Determine desired speeds based on desaturated module states.
        desiredSpeeds = kinematics.toChassisSpeeds(desiredModuleStates);

        // - Create the new state.
        SwerveState newState = new SwerveState(desiredSpeeds, desiredModuleStates);

        // - Set the last state.
        setLastState(newState);

        // - Return the new state.
        return newState;
    }
}
