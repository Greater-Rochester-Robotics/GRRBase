package org.team340.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders.SwerveEncoder;
import org.team340.lib.swerve.hardware.SwerveMotors.SwerveMotor;
import org.team340.lib.util.Math2;

/**
 * An encapsulation of all hardware for a swerve module.
 */
class SwerveModule implements AutoCloseable {

    public final SwerveMotor moveMotor;
    public final SwerveMotor turnMotor;
    public final SwerveEncoder encoder;

    private final SwerveConfig config;
    private final SwerveModuleConfig moduleConfig;
    private final double moveRotationsPerMeter;
    private final boolean hookStatus;

    private final ReadWriteLock cacheMutex = new ReentrantReadWriteLock();
    private final SwerveModulePosition cachedPosition = new SwerveModulePosition();
    private final SwerveModuleState cachedState = new SwerveModuleState();
    private double cachedTurnMotorPosition = 0.0;

    private final SwerveModuleState targetState = new SwerveModuleState();

    /**
     * Create the swerve module.
     * @param config The general swerve API configuration.
     * @param moduleConfig The module's configuration.
     */
    public SwerveModule(SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.config = config;
        this.moduleConfig = moduleConfig;

        moveMotor = moduleConfig.getMoveMotor().apply(config, true);
        turnMotor = moduleConfig.getTurnMotor().apply(config, false);
        encoder = moduleConfig.getEncoder().apply(config, turnMotor);

        moveRotationsPerMeter = config.getMoveGearRatio() / (config.getWheelDiameter() * Math.PI);
        hookStatus = encoder.hookStatus();
    }

    /**
     * Gets the module's configured name.
     */
    public String getName() {
        return moduleConfig.getName();
    }

    /**
     * Returns all Phoenix status signals in use by the module.
     */
    public List<BaseStatusSignal> getSignals() {
        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.addAll(moveMotor.getSignals());
        signals.addAll(turnMotor.getSignals());
        signals.addAll(encoder.getSignals());
        return signals;
    }

    /**
     * Refreshes the cached position and state of the module.
     * @return {@code true} on success, {@code false} if a read error ocurred.
     */
    public boolean refresh() {
        try {
            cacheMutex.writeLock().lock();

            double turnPosition = turnMotor.getPosition();
            double movePosition = moveMotor.getPosition() - (turnPosition * config.getCouplingRatio());
            Rotation2d angle = Rotation2d.fromRotations(hookStatus ? turnPosition : encoder.getPosition());

            cachedPosition.distanceMeters = movePosition / moveRotationsPerMeter;
            cachedPosition.angle = angle;

            cachedState.speedMetersPerSecond = moveMotor.getVelocity() / moveRotationsPerMeter;
            cachedState.angle = angle;

            cachedTurnMotorPosition = turnPosition;

            return moveMotor.readError() || turnMotor.readError() || encoder.readError();
        } finally {
            cacheMutex.writeLock().unlock();
        }
    }

    /**
     * Gets the module's position. The returned {@link SwerveModulePosition}
     * object is final and can be cached, but is volatile.
     */
    public SwerveModulePosition getPosition() {
        return cachedPosition;
    }

    /**
     * Gets the module's state. The returned {@link SwerveModuleState}
     * object is final and can be cached, but is volatile.
     */
    public SwerveModuleState getState() {
        return cachedState;
    }

    /**
     * Gets the module's target state. The returned {@link SwerveModuleState}
     * object is final and can be cached, and is <i>not</i> volatile.
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Sets the target state of the swerve module.
     * @param state The state to apply to the module.
     */
    public void applyState(SwerveModuleState state) {
        try {
            cacheMutex.readLock().lock();
            boolean flipped = false;
            Rotation2d angleDelta = state.angle.minus(getState().angle);
            if (Math.abs(angleDelta.getRadians()) > Math2.HALF_PI) {
                state.speedMetersPerSecond *= -1.0;
                state.angle = state.angle.rotateBy(Rotation2d.kPi);
                flipped = true;
            }

            moveMotor.setVelocity(state.speedMetersPerSecond * moveRotationsPerMeter);

            if (hookStatus) {
                turnMotor.setPosition(state.angle.getRotations());
            } else {
                double optimizedDelta = angleDelta.getRadians() - (flipped ? Math.copySign(Math.PI, angleDelta.getRadians()) : 0.0);
                turnMotor.setPosition(cachedTurnMotorPosition + (Units.radiansToRotations(optimizedDelta) * config.getTurnGearRatio()));
            }

            targetState.speedMetersPerSecond = state.speedMetersPerSecond;
            targetState.angle = state.angle;
        } finally {
            cacheMutex.readLock().unlock();
        }
    }

    /**
     * Drives the swerve module using open-loop voltage. Intended for characterization.
     * @param voltage The voltage to apply to the move motor.
     * @param angle The angle to apply to the turn motor.
     */
    public void applyVoltage(double voltage, Rotation2d angle) {
        try {
            cacheMutex.readLock().lock();
            moveMotor.setVoltage(voltage);
            if (hookStatus) {
                turnMotor.setPosition(angle.getRotations());
            } else {
                turnMotor.setPosition(cachedTurnMotorPosition + (angle.minus(getState().angle).getRotations() * config.getTurnGearRatio()));
            }
        } finally {
            cacheMutex.readLock().unlock();
        }
    }

    @Override
    public void close() {
        try {
            moveMotor.close();
            turnMotor.close();
            encoder.close();
        } catch (Exception e) {}
    }
}
