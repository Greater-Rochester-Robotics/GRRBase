package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.util.command.GRRSubsystem;

/**
 * An example subsystem.
 */
@Logged
public final class Foo extends GRRSubsystem {

    public Foo() {}

    public Command bar() {
        return print("Foo Bar").withName("Foo.bar()");
    }
}
