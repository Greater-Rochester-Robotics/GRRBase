package org.team340.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.team340.lib.util.Alliance;

/**
 * Describes a perspective for {@link ChassisSpeeds}.
 */
public enum ForwardPerspective {
    /**
     * The speeds are relative to the operator's perspective. If the robot
     * is on the blue alliance, X+ drives towards the red alliance. If the
     * robot is on the red alliance, X+ drives towards the blue alliance.
     */
    OPERATOR {
        @Override
        void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            (Alliance.isBlue() ? BLUE_ALLIANCE : RED_ALLIANCE).toRobotSpeeds(speeds, robotAngle);
        }

        @Override
        void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            (Alliance.isBlue() ? BLUE_ALLIANCE : RED_ALLIANCE).toPerspectiveSpeeds(speeds, robotAngle);
        }

        @Override
        Rotation2d getTareRotation() {
            return (Alliance.isBlue() ? BLUE_ALLIANCE : RED_ALLIANCE).getTareRotation();
        }
    },

    /**
     * The speeds are relative to the blue alliance perspective.
     * X+ drives towards the red alliance.
     */
    BLUE_ALLIANCE {
        @Override
        void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            speeds.toRobotRelativeSpeeds(robotAngle);
        }

        @Override
        void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            speeds.toFieldRelativeSpeeds(robotAngle);
        }

        @Override
        Rotation2d getTareRotation() {
            return Rotation2d.kZero;
        }
    },

    /**
     * The speeds are relative to the red alliance perspective.
     * X+ drives towards the blue alliance.
     */
    RED_ALLIANCE {
        @Override
        void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            speeds.toRobotRelativeSpeeds(robotAngle.rotateBy(Rotation2d.kPi));
        }

        @Override
        void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {
            speeds.toFieldRelativeSpeeds(robotAngle.rotateBy(Rotation2d.kPi));
        }

        @Override
        Rotation2d getTareRotation() {
            return Rotation2d.kPi;
        }
    },

    /**
     * The speeds are relative to the robot's perspective.
     * X+ drives forwards relative to the chassis.
     */
    ROBOT {
        @Override
        void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {}

        @Override
        void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle) {}

        @Override
        Rotation2d getTareRotation() {
            // Will no-op downstream.
            return null;
        }
    };

    /**
     * Converts perspective relative speeds to robot relative speeds.
     * @param speeds The perspective relative speeds to convert.
     * @param robotAngle The blue origin relative angle of the robot.
     */
    abstract void toRobotSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle);

    /**
     * Converts robot relative speeds to the perspective relative speeds.
     * @param speeds The robot relative speeds to convert.
     * @param robotAngle The blue origin relative angle of the robot.
     */
    abstract void toPerspectiveSpeeds(ChassisSpeeds speeds, Rotation2d robotAngle);

    /**
     * Gets the rotation to apply as the new zero when
     * taring the robot's rotation to the perspective.
     */
    abstract Rotation2d getTareRotation();
}
