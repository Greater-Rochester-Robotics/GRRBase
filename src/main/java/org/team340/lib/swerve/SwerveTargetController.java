package org.team340.lib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.team340.lib.math.Math2;
import org.team340.lib.swerve.config.SwerveConfig;

// TODO Investigate undesirable behavior while entering and leaving stopped targets with rotational motion

/**
 * Controller that calculates module states based on prior targets while compensating for kinematic constraints as well
 * as globally determining module optimization to shorten and prevent unpredictable behavior during heading transition periods.
 */
public class SwerveTargetController {

    /**
     * A target.
     */
    public static record SwerveTarget(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {}

    /**
     * The general config.
     */
    private final SwerveConfig config;
    /**
     * The kinematics instance in use.
     */
    private final SwerveDriveKinematics kinematics;
    /**
     * The number of modules used by the robot.
     */
    private final int moduleCount;

    /**
     * The last swerve target.
     */
    private SwerveTarget lastTarget;

    /**
     * Create the target controller.
     * @param swerve The swerve subsystem.
     */
    public SwerveTargetController(SwerveBase swerve) {
        this.config = swerve.config;
        this.kinematics = swerve.kinematics;
        this.moduleCount = swerve.moduleTranslations.length;
        this.lastTarget = new SwerveTarget(Math2.CHASSIS_SPEEDS_0, swerve.getModuleStates());
    }

    /**
     * Gets the controller's last target.
     */
    public SwerveTarget getLastTarget() {
        return lastTarget;
    }

    /**
     * Overrides the controller's last target.
     * @param chassisSpeeds The speeds for the target.
     * @param moduleStates The module states for the target.
     */
    public void setLastTarget(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
        setLastTarget(new SwerveTarget(chassisSpeeds, moduleStates));
    }

    /**
     * Overrides the controller's last target.
     * @param target The target to use.
     */
    public void setLastTarget(SwerveTarget target) {
        lastTarget = target;
    }

    /**
     * Calculates a new target.
     * @param desiredSpeeds The new desired chassis speeds.
     * @return The new target.
     */
    public SwerveTarget calculate(ChassisSpeeds desiredSpeeds) {
        // - Determine the desired swerve module states.
        SwerveModuleState[] desiredModuleStates = kinematics.toSwerveModuleStates(desiredSpeeds);

        // - Preliminary wheel speed desaturation based on the configured max robot velocity.
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, config.getMaxV());

        // - Determine desired speeds based on desaturated module states.
        desiredSpeeds = kinematics.toChassisSpeeds(desiredModuleStates);

        // - Declare a list for modules that require special behavior for their
        //   heading outside of the kinematics calculated heading.
        List<Optional<Rotation2d>> overrideHeading = new ArrayList<>(moduleCount);

        // - If the desired speed is a stop.
        boolean desireStop = Math2.twist2dEpsilonEquals(
            new Twist2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond, desiredSpeeds.omegaRadiansPerSecond),
            Math2.TWIST2D_0
        );
        // (Special behavior for module rotation when the desired chassis speeds is 0)
        // If the desired target has no movement:
        if (desireStop) {
            // - Find the last target's total velocity across the X and Y axis.
            Translation2d lastV = new Translation2d(lastTarget.chassisSpeeds.vxMetersPerSecond, lastTarget.chassisSpeeds.vyMetersPerSecond);

            // - Determine if the last target had a velocity of 0.
            boolean wasMoving = !Math2.epsilonEquals(lastV.getNorm(), 0.0);

            // For every module:
            for (int i = 0; i < moduleCount; ++i) {
                // - If the robot was moving, set the desired angle to the direction of the
                //   robot's current lateral motion. This is to prevent the robot from veering
                //   if it is stopping from a state of rotating while translating.
                desiredModuleStates[i].angle = wasMoving ? lastV.getAngle() : lastTarget.moduleStates[i].angle;

                // - Translational speed is still 0.
                desiredModuleStates[i].speedMetersPerSecond = 0.0;

                // - Add the heading to overrides.
                overrideHeading.add(Optional.of(desiredModuleStates[i].angle));
            }
        }

        // - Declare arrays to be hydrated with calculated module velocities and headings.
        double[] modulesLastVx = new double[moduleCount];
        double[] modulesLastVy = new double[moduleCount];
        Rotation2d[] modulesLastHeading = new Rotation2d[moduleCount];
        double[] modulesDesiredVx = new double[moduleCount];
        double[] modulesDesiredVy = new double[moduleCount];
        Rotation2d[] modulesDesiredHeading = new Rotation2d[moduleCount];

        // - Declare a tripwire to determine if all modules are flipping with the
        //   desired speeds (indicative of moving in the opposite direction within
        //   90 degrees). If this remains true, it is probably faster to start
        //   over with a velocity of 0.
        boolean allFlip = true;

        // (Special behavior for translation direction changes in the
        // opposite direction of current movement within 90 degrees)
        // For every module:
        for (int i = 0; i < moduleCount; ++i) {
            // - Find the last module velocity and heading using the last module states.
            modulesLastVx[i] = lastTarget.moduleStates[i].angle.getCos() * lastTarget.moduleStates[i].speedMetersPerSecond;
            modulesLastVy[i] = lastTarget.moduleStates[i].angle.getSin() * lastTarget.moduleStates[i].speedMetersPerSecond;
            modulesLastHeading[i] = lastTarget.moduleStates[i].angle;

            // If the last module velocity was in reverse:
            //     - Flip the heading.
            if (lastTarget.moduleStates[i].speedMetersPerSecond < 0.0) {
                modulesLastHeading[i] = Rotation2d.fromRadians(MathUtil.angleModulus(modulesLastHeading[i].getRadians() + Math.PI));
            }

            // - Find the desired module velocity and heading using the desired module states.
            modulesDesiredVx[i] = desiredModuleStates[i].angle.getCos() * desiredModuleStates[i].speedMetersPerSecond;
            modulesDesiredVy[i] = desiredModuleStates[i].angle.getSin() * desiredModuleStates[i].speedMetersPerSecond;
            modulesDesiredHeading[i] = desiredModuleStates[i].angle;

            // If the desired module velocity is in reverse:
            //     - Flip the heading.
            if (desiredModuleStates[i].speedMetersPerSecond < 0.0) {
                modulesDesiredHeading[i] = Rotation2d.fromRadians(MathUtil.angleModulus(modulesDesiredHeading[i].getRadians() + Math.PI));
            }

            // If the desired target doesn't require a flip and the allFlip tripwire has not been hit:
            //     - Set the allFlip tripwire to false.
            if (allFlip && Math.abs(modulesLastHeading[i].times(-1.0).rotateBy(modulesDesiredHeading[i]).getRadians()) < Math2.HALF_PI) {
                allFlip = false;
            }
        }

        // If the allFlip tripwire has been hit, and the last and desired target contains movement:
        //     - Start over with a desired speed of 0 (this should be faster).
        if (
            allFlip &&
            !desireStop &&
            !Math2.twist2dEpsilonEquals(
                new Twist2d(
                    lastTarget.chassisSpeeds.vxMetersPerSecond,
                    lastTarget.chassisSpeeds.vyMetersPerSecond,
                    lastTarget.chassisSpeeds.omegaRadiansPerSecond
                ),
                Math2.TWIST2D_0
            )
        ) {
            return calculate(Math2.CHASSIS_SPEEDS_0);
        }

        // - Declare the velocity delta scalar. This is an interpolation between the desired speeds and the last target's speeds. It
        //   will be lowered based on kinematic constraints. Applied to the final chassis speeds and module states before
        //   they are overridden.
        double dvScalar = 1.0;

        // - Find the maximum feasible delta in the a module's heading.
        //   Acceleration is assumed to be a non-factor.
        double maxHeadingDelta = config.getPeriod() * config.getMaxModuleRv();

        // (Limit the velocity delta based on module heading constraints)
        // If we aren't stopping:
        if (!desireStop) {
            // For every module:
            for (int i = 0; i < moduleCount; ++i) {
                // - Add an empty Optional to the override heading array (this is to ensure null values are not retrieved).
                overrideHeading.add(Optional.empty());

                // If the last target's velocity is 0:
                if (Math2.epsilonEquals(lastTarget.moduleStates[i].speedMetersPerSecond, 0.0)) {
                    // If the desired target's velocity is 0:
                    //     - Add the module to the overrideHeading list with the last target's heading and skip the rest of the loop.
                    if (Math2.epsilonEquals(desiredModuleStates[i].speedMetersPerSecond, 0.0)) {
                        overrideHeading.set(i, Optional.of(lastTarget.moduleStates[i].angle));
                        continue;
                    }

                    // - Find the rotational delta between the last and desired module heading.
                    Rotation2d rotDelta = lastTarget.moduleStates[i].angle.times(-1.0).rotateBy(desiredModuleStates[i].angle);

                    // If the rotational delta is greater than PI / 2 (90 degrees) and should be optimized:
                    //     - Rotate the delta to an optimized position.
                    if (flipHeading(rotDelta)) rotDelta = rotDelta.rotateBy(Math2.ROTATION2D_PI);

                    // If the desired target will take a single periodic loop to achieve:
                    //     - Add the module to the overrideHeading list with the desired target's
                    //       heading and skip the rest of the loop. Calculating a reduction for the
                    //       velocity delta shouldn't result in a meaningful change.
                    //     Else:
                    //         - Add the module to the overrideHeading list with a
                    //           heading achievable in the next periodic loop.
                    //         - Velocity change is now 0, as we wait for all modules to move before
                    //           accelerating from a stop. This prevents the "wiggle" typically seen in
                    //           swerve drive after accelerating from a dead stop.
                    if (Math.abs(rotDelta.getRadians()) / maxHeadingDelta <= 1.0) {
                        overrideHeading.set(i, Optional.of(desiredModuleStates[i].angle));
                        continue;
                    } else {
                        overrideHeading.set(
                            i,
                            Optional.of(
                                lastTarget.moduleStates[i].angle.rotateBy(
                                        Rotation2d.fromRadians(Math.signum(rotDelta.getRadians()) * maxHeadingDelta)
                                    )
                            )
                        );
                        dvScalar = 0.0;
                        continue;
                    }
                }

                // - If the velocity delta scalar is 0, skip the rest of the loop.
                if (dvScalar == 0.0) continue;

                // - Find the module's velocity delta scalar from its heading delta (refer to the method below on how this is derived).
                double moduleDvScalar = getModuleHeadingVDS(
                    modulesLastVx[i],
                    modulesLastVy[i],
                    modulesLastHeading[i].getRadians(),
                    modulesDesiredVx[i],
                    modulesDesiredVy[i],
                    modulesDesiredHeading[i].getRadians(),
                    maxHeadingDelta
                );

                // - Set the scalar to the minimum of its current value and the
                //   calculated maximum feasible scalar from the module.
                dvScalar = Math.min(dvScalar, moduleDvScalar);
            }
        }

        // (Limit the velocity delta scalar based on module velocity constraints)
        // For every module:
        for (int i = 0; i < moduleCount; ++i) {
            // If the scalar is already 0:
            //     - Exit.
            if (dvScalar == 0.0) break;

            // - Calculate the desaturated x and y velocity based on the
            //   current velocity delta scalar.
            double desiredModuleVx = dvScalar == 1.0
                ? modulesDesiredVx[i]
                : ((modulesDesiredVx[i] - modulesLastVx[i]) * dvScalar) + modulesLastVx[i];
            double desiredModuleVy = dvScalar == 1.0
                ? modulesDesiredVy[i]
                : ((modulesDesiredVy[i] - modulesLastVy[i]) * dvScalar) + modulesLastVy[i];

            // - Find the module's velocity delta scalar from its velocity delta (refer to the method below on how this is derived).
            double moduleDvScalar =
                dvScalar *
                getModuleVelocityVDS(
                    modulesLastVx[i],
                    modulesLastVy[i],
                    desiredModuleVx,
                    desiredModuleVy,
                    config.getPeriod() * config.getMaxA()
                );

            // - Set the scalar to the minimum of its current value and the
            //   calculated maximum feasible scalar from the module.
            dvScalar = Math.min(dvScalar, moduleDvScalar);
        }

        // - Declare the constrained speeds using the velocity delta scalar.
        ChassisSpeeds constrainedSpeeds = new ChassisSpeeds(
            lastTarget.chassisSpeeds.vxMetersPerSecond +
            (dvScalar * (desiredSpeeds.vxMetersPerSecond - lastTarget.chassisSpeeds.vxMetersPerSecond)),
            lastTarget.chassisSpeeds.vyMetersPerSecond +
            (dvScalar * (desiredSpeeds.vyMetersPerSecond - lastTarget.chassisSpeeds.vyMetersPerSecond)),
            lastTarget.chassisSpeeds.omegaRadiansPerSecond +
            (dvScalar * (desiredSpeeds.omegaRadiansPerSecond - lastTarget.chassisSpeeds.omegaRadiansPerSecond))
        );

        // - Compute module states from constrained speeds..
        SwerveModuleState[] constrainedStates = kinematics.toSwerveModuleStates(constrainedSpeeds);

        // (Module overrides / "global" heading optimization)
        // For every module:
        for (int i = 0; i < moduleCount; ++i) {
            // - Get the module's override.
            Optional<Rotation2d> couldOverride = overrideHeading.get(i);

            // If the module is overridden:
            if (couldOverride.isPresent()) {
                // - Get the override.
                Rotation2d override = couldOverride.get();

                // - If the override is a flipped heading, reverse the module's speed.
                if (flipHeading(constrainedStates[i].angle.times(-1.0).rotateBy(override))) {
                    constrainedStates[i].speedMetersPerSecond *= -1.0;
                }

                // - Set the module to the overridden angle.
                constrainedStates[i].angle = override;
            }

            // - Find the module's change in heading.
            Rotation2d deltaRotation = lastTarget.moduleStates[i].angle.times(-1.0).rotateBy(constrainedStates[i].angle);

            // If the change in heading is greater than 90 degrees:
            if (flipHeading(deltaRotation)) {
                // - Optimize the heading and reverse the module's speed.
                constrainedStates[i].angle =
                    Rotation2d.fromRadians(MathUtil.angleModulus(constrainedStates[i].angle.getRadians() + Math.PI));
                constrainedStates[i].speedMetersPerSecond *= -1.0;
            }
        }

        // - Create the new target.
        SwerveTarget newTarget = new SwerveTarget(constrainedSpeeds, constrainedStates);
        // - Set the last target.
        setLastTarget(newTarget);
        // - Return the new target.
        return newTarget;
    }

    /**
     * If a heading should be optimized.
     * @param heading The heading to check.
     */
    private boolean flipHeading(Rotation2d heading) {
        return Math.abs(heading.getRadians()) > Math2.HALF_PI;
    }

    /**
     * Calculates the velocity delta scalar (percent of difference between last and desired target) from a module's heading.
     * @param lastVx Last target {@code x} velocity.
     * @param lastVy Last target {@code y} velocity.
     * @param lastHeading Last target heading.
     * @param desiredVx Desired {@code x} velocity.
     * @param desiredVy Desired {@code y} velocity.
     * @param desiredHeading Desired heading.
     * @param maxRotationalVelocityStep The maximum allowed difference in rotational velocity in a periodic loop iteration.
     */
    private double getModuleHeadingVDS(
        double lastVx,
        double lastVy,
        double lastHeading,
        double desiredVx,
        double desiredVy,
        double desiredHeading,
        double maxRotationalVelocityStep
    ) {
        // - Make sure the headings are optimized.
        desiredHeading = Math2.wrapAbout(lastHeading, desiredHeading);

        // - Find the difference between the last and desired headings.
        double diff = desiredHeading - lastHeading;

        // - If the last and desired heading is achievable in one
        //   periodic loop, return 1 as no interpolation is needed.
        if (Math.abs(diff) <= maxRotationalVelocityStep) return 1.0;

        // - Find an achievable heading.
        double achievableHeading = lastHeading + Math.signum(diff) * maxRotationalVelocityStep;

        // - Describes the following function:
        //     - x is the module's X velocity relative to the robot.
        //     - y is the module's Y velocity relative to the robot.
        //     Returns:
        //         - Get the module's velocity as a heading.
        //         - Localize the robot's velocity heading around the last module heading.
        //         - Subtract the achievable heading.
        Math2.Parametric func = (x, y) -> Math2.wrapAbout(lastHeading, Math.atan2(y, x)) - achievableHeading;

        // - Solves the above function for its root within the bounds of the last
        //   and desired chassis speeds. Colloquially, this solves for a percentage to
        //   reduce the robot's velocity by to ensure that undesirable behavior does not
        //   arise from modules pulling the robot in unpredictable directions during
        //   the heading transitional period.
        return Math2.findRoot(
            func,
            lastVx,
            lastVy,
            lastHeading - achievableHeading,
            desiredVx,
            desiredVy,
            desiredHeading - achievableHeading,
            6
        );
    }

    /**
     * Calculates the velocity delta scalar (percent of difference between last and desired target) from a module's velocity.
     * @param lastVx Last target {@code x} velocity.
     * @param lastVy Last target {@code y} velocity.
     * @param desiredVx Desired {@code x} velocity.
     * @param desiredVy Desired {@code y} velocity.
     * @param maxVelocityStep The maximum allowed difference in velocity in a periodic loop iteration.
     */
    private double getModuleVelocityVDS(double lastVx, double lastVy, double desiredVx, double desiredVy, double maxVelocityStep) {
        // - Compute the last and desired translational velocities.
        double lastNorm = Math.hypot(lastVx, lastVy);
        double desiredNorm = Math.hypot(desiredVx, desiredVy);

        // - Find the difference between the last and desired headings.
        double diff = desiredNorm - lastNorm;

        // - If the last and desired velocity is achievable in one
        //   periodic loop, return 1 as no interpolation is needed.
        if (Math.abs(diff) <= maxVelocityStep) return 1.0;

        // - Find an achievable velocity.
        double achievableVelocity = lastNorm + Math.signum(diff) * maxVelocityStep;

        // - Describes the following function:
        //     - x is the module's X velocity relative to the robot.
        //     - y is the module's Y velocity relative to the robot.
        //     Returns:
        //         - Find the total velocity.
        //         - Take the difference of the achievable velocity.
        Math2.Parametric func = (x, y) -> Math.hypot(x, y) - achievableVelocity;

        // - Solves the above function for its root within the bounds of the
        //   last and desired velocity. This is used to prevent infeasible
        //   acceleration while translating on a per module basis.
        return Math2.findRoot(
            func,
            lastVx,
            lastVy,
            lastNorm - achievableVelocity,
            desiredVx,
            desiredVy,
            desiredNorm - achievableVelocity,
            10
        );
    }
}
