package org.team340.lib.util.redux;

import com.reduxrobotics.frames.DoubleFrame;
import com.reduxrobotics.frames.Frame;
import com.reduxrobotics.sensors.canandgyro.QuaternionFrame;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class ReduxUtil {

    private ReduxUtil() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * Performs latency compensation on the yaw of the provided quaternion frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double getLatencyCompensatedYaw(QuaternionFrame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getYaw();
            timestamp = frame.getTimestamp();
        }

        return getLatencyCompensatedValue(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the pitch of the provided quaternion frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double getLatencyCompensatedPitch(QuaternionFrame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getPitch();
            timestamp = frame.getTimestamp();
        }

        return getLatencyCompensatedValue(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the roll of the provided quaternion frame
     * using the {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double getLatencyCompensatedRoll(QuaternionFrame frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getRoll();
            timestamp = frame.getTimestamp();
        }

        return getLatencyCompensatedValue(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the provided frame using the
     * {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double getLatencyCompensatedValue(DoubleFrame<Double> frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getData();
            timestamp = frame.getTimestamp();
        }

        return getLatencyCompensatedValue(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the provided frame using the
     * {@code slope} to determine the magnitude of the compensation.
     * @param frame The frame to compensate.
     * @param slope The rate of change of the frame's data.
     */
    public static double getLatencyCompensatedValue(Frame<Double> frame, double slope) {
        double value, timestamp;
        synchronized (frame) {
            value = frame.getValue();
            timestamp = frame.getTimestamp();
        }

        return getLatencyCompensatedValue(value, timestamp, slope);
    }

    /**
     * Performs latency compensation on the provided frame using the
     * {@code slope} to determine the magnitude of the compensation.
     * @param value The value of the frame.
     * @param timestamp The timestamp of the frame.
     * @param slope The rate of change of the frame's data.
     */
    public static double getLatencyCompensatedValue(double value, double timestamp, double slope) {
        double latency = MathUtil.clamp(Timer.getFPGATimestamp() - timestamp, 0.0, 0.2);
        return value + (slope * latency);
    }
}
