package org.team340.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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

    private final Lock cacheMutex = new ReentrantLock();
    private final SwerveModulePosition cachedPosition = new SwerveModulePosition();
    private final SwerveModuleState cachedState = new SwerveModuleState();
    private double cachedTurnMotorPosition = 0.0;

    private final SwerveModuleState nextTarget = new SwerveModuleState();
    private final SwerveModuleState lastTarget = new SwerveModuleState();

    /**
     * Create the swerve module.
     * @param config The general swerve API configuration.
     * @param moduleConfig The module's configuration.
     */
    public SwerveModule(SwerveConfig config, SwerveModuleConfig moduleConfig) {
        this.config = config;
        this.moduleConfig = moduleConfig;

        moveMotor = SwerveMotor.construct(moduleConfig.moveMotor, config, true);
        turnMotor = SwerveMotor.construct(moduleConfig.turnMotor, config, false);
        encoder = SwerveEncoder.construct(moduleConfig.encoder, config, turnMotor);

        moveRotationsPerMeter = config.moveGearRatio / (config.wheelDiameter * Math.PI);
        hookStatus = encoder.hookStatus();
    }

    /**
     * Gets the module's configured name.
     */
    public String getName() {
        return moduleConfig.name;
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
        cacheMutex.lock();
        try {
            double turnPosition = turnMotor.getPosition();
            double movePosition = moveMotor.getPosition() - (turnPosition * config.couplingRatio);
            Rotation2d angle = Rotation2d.fromRotations(hookStatus ? turnPosition : encoder.getPosition());

            cachedPosition.distanceMeters = movePosition / moveRotationsPerMeter;
            cachedPosition.angle = angle;

            cachedState.speedMetersPerSecond = moveMotor.getVelocity() / moveRotationsPerMeter;
            cachedState.angle = angle;

            cachedTurnMotorPosition = turnPosition;

            return !moveMotor.readError() && !turnMotor.readError() && !encoder.readError();
        } finally {
            cacheMutex.unlock();
        }
    }

    /**
     * Gets the module's position. The returned {@link SwerveModulePosition}
     * object is final and can be cached, but is volatile, as it may be
     * asynchronously refreshed by the odometry thread.
     */
    public SwerveModulePosition getPosition() {
        return cachedPosition;
    }

    /**
     * Gets the module's state. The returned {@link SwerveModuleState}
     * object is final and can be cached, but is volatile, as it may
     * be asynchronously refreshed by the odometry thread.
     */
    public SwerveModuleState getState() {
        return cachedState;
    }

    /**
     * Gets the module's next target state. The returned {@link SwerveModuleState}
     * object is final and can be cached, and is <i>not</i> volatile.
     */
    public SwerveModuleState getNextTarget() {
        return nextTarget;
    }

    /**
     * Gets the module's last target state. The returned {@link SwerveModuleState}
     * object is final and can be cached, and is <i>not</i> volatile.
     */
    public SwerveModuleState getLastTarget() {
        return lastTarget;
    }

    /**
     * Sets the target state of the swerve module.
     * @param state The state to apply to the module.
     */
    public void applyState(SwerveModuleState state) {
        cacheMutex.lock();
        try {
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
                double optimizedDelta =
                    angleDelta.getRadians() - (flipped ? Math.copySign(Math.PI, angleDelta.getRadians()) : 0.0);
                turnMotor.setPosition(
                    cachedTurnMotorPosition + (Units.radiansToRotations(optimizedDelta) * config.turnGearRatio)
                );
            }

            Math2.copyInto(nextTarget, lastTarget);
            Math2.copyInto(state, nextTarget);
        } finally {
            cacheMutex.unlock();
        }
    }

    /**
     * Drives the swerve module using open-loop voltage. Intended for characterization.
     * @param voltage The voltage to apply to the move motor.
     * @param angle The angle to apply to the turn motor.
     */
    public void applyVoltage(double voltage, Rotation2d angle) {
        cacheMutex.lock();
        try {
            moveMotor.setVoltage(voltage);
            if (hookStatus) {
                turnMotor.setPosition(angle.getRotations());
            } else {
                turnMotor.setPosition(
                    cachedTurnMotorPosition + (angle.minus(getState().angle).getRotations() * config.turnGearRatio)
                );
            }
        } finally {
            cacheMutex.unlock();
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
