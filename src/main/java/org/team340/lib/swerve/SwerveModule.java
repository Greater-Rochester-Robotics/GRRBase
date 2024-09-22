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
public class SwerveModule implements AutoCloseable {

    private final SwerveConfig config;
    private final SwerveModuleConfig moduleConfig;
    final SwerveMotor moveMotor;
    final SwerveMotor turnMotor;
    final SwerveEncoder encoder;

    private final double moveRotationsPerMeter;
    private final boolean hookStatus;

    private final SwerveModuleState targetState = new SwerveModuleState();

    private final ReadWriteLock cacheMutex = new ReentrantReadWriteLock();
    private SwerveModulePosition cachedPosition = new SwerveModulePosition();
    private SwerveModuleState cachedState = new SwerveModuleState();
    private double cachedTurnMotorPosition = 0.0;

    /**
     * Create the swerve module.
     * @param config The general swerve API configuration.
     * @param moduleConfig The module's configuration.
     */
    SwerveModule(SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.config = config;
        this.moduleConfig = moduleConfig;

        moveMotor = moduleConfig.getMoveMotor().apply(config, true);
        turnMotor = moduleConfig.getTurnMotor().apply(config, false);
        encoder = moduleConfig.getEncoder().apply(config, turnMotor);

        moveRotationsPerMeter = config.getMoveGearRatio() / (config.getWheelDiameter() * Math.PI);
        hookStatus = encoder.hookStatus();
    }

    /**
     * Refreshes the cached position and state of the module.
     * @return {@code true} on success, {@code false} if a read error ocurred.
     */
    boolean refresh() {
        try {
            cacheMutex.writeLock().lock();

            double turnPosition = turnMotor.getPosition();
            double movePosition = moveMotor.getPosition() - (turnPosition * config.getCouplingRatio());
            Rotation2d angle = Rotation2d.fromRotations(hookStatus ? turnPosition : encoder.getPosition());

            cachedPosition = new SwerveModulePosition(movePosition / moveRotationsPerMeter, angle);
            cachedState = new SwerveModuleState(moveMotor.getVelocity() / moveRotationsPerMeter, angle);
            cachedTurnMotorPosition = turnPosition;

            return moveMotor.readError() || turnMotor.readError() || encoder.readError();
        } finally {
            cacheMutex.writeLock().unlock();
        }
    }

    /**
     * Returns all Phoenix status signals in use by the module.
     */
    List<BaseStatusSignal> getSignals() {
        List<BaseStatusSignal> signals = new ArrayList<>();
        signals.addAll(moveMotor.getSignals());
        signals.addAll(turnMotor.getSignals());
        signals.addAll(encoder.getSignals());
        return signals;
    }

    /**
     * Gets the module's configured name.
     */
    public String getName() {
        return moduleConfig.getName();
    }

    /**
     * Gets the module's position.
     */
    public SwerveModulePosition getPosition() {
        try {
            cacheMutex.readLock().lock();
            return cachedPosition;
        } finally {
            cacheMutex.readLock().unlock();
        }
    }

    /**
     * Gets the module's state.
     */
    public SwerveModuleState getState() {
        try {
            cacheMutex.readLock().lock();
            return cachedState;
        } finally {
            cacheMutex.readLock().unlock();
        }
    }

    /**
     * Gets the module's target state.
     */
    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * Sets the target state of the swerve module.
     * @param state The state to apply to the module.
     */
    public void setState(SwerveModuleState state) {
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
     * Drives the swerve module using voltage. Intended for characterization.
     * @param voltage The voltage to apply to the move motor.
     * @param angle The angle to apply to the turn motor.
     */
    public void setVoltage(double voltage, Rotation2d angle) {
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
