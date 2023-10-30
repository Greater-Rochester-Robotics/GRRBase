package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is used to declare autonomous routines.
 */
public class Autos {

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * An example auto.
     */
    public static Command example() {
        return sequence(swerve.drive(() -> 0.1, () -> 0.0, () -> 0.0, true).withTimeout(1.0));
    }
}
