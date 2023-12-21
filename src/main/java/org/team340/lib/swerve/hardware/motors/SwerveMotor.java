package org.team340.lib.swerve.hardware.motors;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.team340.lib.swerve.SwerveModule;

/**
 * A motor for swerve, can be a move or turn motor.
 * Bound to {@link SwerveModule}s.
 */
public abstract class SwerveMotor {

    private final boolean isMoveMotor;
    private final Timer simTimer;
    private double simTarget = 0.0;
    private double simPosition = 0.0;

    /**
     * Create the swerve motor.
     * @param isMoveMotor If the motor is a move motor.
     */
    public SwerveMotor(boolean isMoveMotor) {
        this.isMoveMotor = isMoveMotor;

        if (RobotBase.isSimulation()) {
            simTimer = new Timer();
        } else {
            simTimer = null;
        }
    }

    /**
     * Gets the motor's velocity in meters/second.
     */
    protected abstract double getRealVelocity();

    /**
     * Gets the motor's relative position in radians.
     */
    protected abstract double getRealPosition();

    /**
     * Sets the motor's closed loop target.
     * If the motor is a move motor, the target is in meters/second.
     * If the motor is a turn motor, the target is an unclamped position in radians.
     * @param target The target.
     * @param ff Arbitrary feed forward.
     */
    protected abstract void setRealReference(double target, double ff);

    /**
     * If the motor is a move motor.
     */
    public final boolean isMoveMotor() {
        return isMoveMotor;
    }

    /**
     * Gets the motor's velocity in meters/second.
     * If the motor is being ran in simulation, the last commanded velocity is returned.
     * {@code 0.0} is returned in simulation if the motor is a turn motor.
     */
    public final double getVelocity() {
        if (RobotBase.isSimulation()) {
            if (isMoveMotor) return simTarget; else return 0.0;
        } else {
            return getRealVelocity();
        }
    }

    /**
     * Gets the motor's relative position in radians.
     * If the motor is being ran in simulation, the last commanded position is returned.
     */
    public final double getPosition() {
        if (RobotBase.isSimulation()) {
            return simPosition;
        } else {
            return getRealVelocity();
        }
    }

    /**
     * Sets the motor's closed loop target.
     * If the motor is a move motor, the target is in meters/second.
     * If the motor is a turn motor, the target is an unclamped position in radians.
     * @param target The target.
     * @param ff Arbitrary feed forward.
     */
    public final void setReference(double target, double ff) {
        if (RobotBase.isSimulation()) {
            if (isMoveMotor) simPosition += this.simTarget * simTimer.get(); else simPosition = target;
            this.simTarget = target;
            simTimer.restart();
        }

        setRealReference(target, ff);
    }
}
