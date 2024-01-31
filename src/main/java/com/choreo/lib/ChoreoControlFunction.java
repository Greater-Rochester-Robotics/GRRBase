// Copyright (c) Choreo contributors
// From Choreolib@https://github.com/SleipnirGroup/Choreo/commit/09b94f7b24969b4ca3d910439f0be1fb562834d5

package com.choreo.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.BiFunction;

/**
 * This interface exists as a type alias. A ChoreoControlFunction has signature (Pose2d currentPose,
 * ChoreoTrajectoryState referenceState)-&gt;ChassisSpeeds, where the function returns
 * robot-relative ChassisSpeeds for the robot.
 */
public interface ChoreoControlFunction extends BiFunction<Pose2d, ChoreoTrajectoryState, ChassisSpeeds> {}
