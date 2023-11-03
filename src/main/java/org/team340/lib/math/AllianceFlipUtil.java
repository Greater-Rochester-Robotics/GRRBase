package org.team340.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility class for flipping coordinate values based on the robot's current alliance. Provided values
 * should be for the blue alliance, and are flipped to the red alliance if the driver station reports
 * the robot to be on the red alliance. Alternatively, you can provide the robot's position, and the
 * returned values will be coordinates for the blue alliance.
 *
 * <p>This implementation is specific to the 2023 game. Coordinate flipping is likely to change between
 * seasons, so this class should be revisited at the start of the season.
 *
 * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
 */
public class AllianceFlipUtil {

    private final double fieldWidth;

    /**
     * Create the flip util.
     * @param fieldWidth The field's width in meters.
     */
    public AllianceFlipUtil(double fieldWidth) {
        this.fieldWidth = fieldWidth;
    }

    /**
     * Flip a {@link Pose2d}.
     * @param pose The original pose.
     * @return The pose flipped for the appropriate alliance.
     */
    public Pose2d flip(Pose2d pose) {
        return shouldFlip() ? new Pose2d(flip(pose.getTranslation()), pose.getRotation()) : pose;
    }

    /**
     * Flip a {@link Translation2d}.
     * @param translation The original translation.
     * @return The translation flipped for the appropriate alliance.
     */
    public Translation2d flip(Translation2d translation) {
        return shouldFlip() ? new Translation2d(translation.getX(), fieldWidth - translation.getY()) : translation;
    }

    /**
     * Flip a Y value.
     * @param y The Y value to flip in meters.
     * @return The Y value flipped for the appropriate alliance.
     */
    public double flip(double y) {
        return shouldFlip() ? fieldWidth - y : y;
    }

    private static boolean shouldFlip() {
        return !DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue);
    }
}
