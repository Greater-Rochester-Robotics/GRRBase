package org.team340.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.ArrayList;
import java.util.List;
import org.team340.lib.math.Math2;

/**
 * An extension of WPILib's {@link Field2d} with a helper method easily update the visualization.
 */
public class SwerveField2d extends Field2d {

    private final SwerveBase swerve;

    /**
     * Create the field.
     * @param swerve The swerve subsystem.
     */
    public SwerveField2d(SwerveBase swerve) {
        this.swerve = swerve;
    }

    /**
     * Update the field.
     */
    public void update() {
        Pose2d newPose = swerve.getPosition();
        if (!DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue)) {
            newPose =
                new Pose2d(
                    swerve.config.getFieldLength() - newPose.getX(),
                    swerve.config.getFieldWidth() - newPose.getY(),
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
        for (int i = 0; i < swerve.moduleTranslations.length; i++) {
            Translation2d translation = swerve.moduleTranslations[i].rotateBy(newPose.getRotation()).plus(newPose.getTranslation());
            modulePoses.add(
                new Pose2d(
                    Math2.toFixed(translation.getX()),
                    Math2.toFixed(translation.getY()),
                    Rotation2d.fromDegrees(
                        Math2.toFixed(Rotation2d.fromRadians(swerve.modules[i].getAngle()).plus(newPose.getRotation()).getDegrees())
                    )
                )
            );
        }

        getObject("Modules").setPoses(modulePoses);
    }
}
