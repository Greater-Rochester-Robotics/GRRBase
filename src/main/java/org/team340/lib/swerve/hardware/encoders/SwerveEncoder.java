package org.team340.lib.swerve.hardware.encoders;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import org.team340.lib.swerve.SwerveBase.SwerveAbsoluteEncoderType;

/**
 * An absolute encoder wrapper for swerve.
 * Bound to {@link SwerveModule}s.
 */
public abstract class SwerveEncoder {

    private final SwerveAbsoluteEncoderType type;
    private double simPosition = 0.0;

    public SwerveEncoder(SwerveAbsoluteEncoderType type) {
        this.type = type;
    }

    /**
     * Gets the encoder's position in radians.
     * Clamped from {@code -PI} to {@code PI}.
     */
    protected abstract double getRealPosition();

    /**
     * Gets the encoder's type.
     */
    public final SwerveAbsoluteEncoderType getType() {
        return type;
    }

    /**
     * Gets the encoder's absolute position in radians.
     * Clamped from {@code -PI} to {@code PI}.
     */
    public final double getPosition() {
        if (RobotBase.isSimulation()) {
            return simPosition;
        } else {
            return getRealPosition();
        }
    }

    /**
     * Sets the simulated encoder's position.
     * Has no effect when ran on a real robot.
     * @param position The new position in radians. Does not need to be clamped.
     */
    public final void setSimPosition(double position) {
        if (!RobotBase.isSimulation()) return;
        simPosition = MathUtil.angleModulus(position);
    }
}
