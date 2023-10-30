package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is used to declare the systems check command.
 */
public class SystemsCheck {

    private SystemsCheck() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * The systems check command.
     * Runs all of the robot's mechanisms.
     */
    public static Command command() {
        return sequence(swerve.drive(() -> 0.1, () -> 0.0, () -> 0.0, true).withTimeout(1.0));
    }
}
