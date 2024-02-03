package org.team340.lib.swerve.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.util.Math2;

/**
 * An extension of WPILib's {@link Field2d} with a helper method easily update the visualization.
 */
public class SwerveField2d extends Field2d {

    private final SwerveConfig config;

    /**
     * Create the field.
     * @param config The general swerve config.
     * @param moduleTranslations Translations representing the locations of the swerve modules.
     */
    public SwerveField2d(SwerveConfig config) {
        this.config = config;
    }

    /**
     * Update the field.
     * @param newPose The robot's new pose.
     */
    public void update(Pose2d newPose) {
        if (!DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
            newPose =
                new Pose2d(
                    config.getFieldLength() - newPose.getX(),
                    config.getFieldWidth() - newPose.getY(),
                    newPose.getRotation().minus(Math2.ROTATION2D_PI)
                );
        }

        setRobotPose(
            new Pose2d(
                Math2.toFixed(newPose.getX()),
                Math2.toFixed(newPose.getY()),
                Rotation2d.fromDegrees(Math2.toFixed(newPose.getRotation().getDegrees()))
            )
        );
    }
}
