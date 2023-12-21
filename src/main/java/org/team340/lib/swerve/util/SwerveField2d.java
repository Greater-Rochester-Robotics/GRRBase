package org.team340.lib.swerve.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.util.Math2;

/**
 * An extension of WPILib's {@link Field2d} with a helper method easily update the visualization.
 */
public class SwerveField2d extends Field2d {

    private final SwerveConfig config;
    private final Translation2d[] moduleTranslations;

    /**
     * Create the field.
     * @param config The general swerve config.
     * @param moduleTranslations Translations representing the locations of the swerve modules.
     */
    public SwerveField2d(SwerveConfig config, Translation2d[] moduleTranslations) {
        this.config = config;
        this.moduleTranslations = moduleTranslations;
    }

    /**
     * Update the field.
     * @param newPose The robot's new pose.
     * @param modulePositions The robot's module positions.
     */
    public void update(Pose2d newPose, SwerveModulePosition[] modulePositions) {
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

        List<Pose2d> modulePoses = new ArrayList<>();
        for (int i = 0; i < moduleTranslations.length; i++) {
            Translation2d translation = moduleTranslations[i].rotateBy(newPose.getRotation()).plus(newPose.getTranslation());
            modulePoses.add(
                new Pose2d(
                    Math2.toFixed(translation.getX()),
                    Math2.toFixed(translation.getY()),
                    Rotation2d.fromDegrees(Math2.toFixed(modulePositions[i].angle.plus(newPose.getRotation()).getDegrees()))
                )
            );
        }

        getObject("Modules").setPoses(modulePoses);
    }
}
