package org.team340.lib.util;

import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class GRRSubsystem implements Subsystem {

    public GRRSubsystem() {
        register();
    }

    /**
     * Creates a command builder that requires this subsystem.
     */
    protected CommandBuilder commandBuilder() {
        return new CommandBuilder(this);
    }

    /**
     * Creates a command builder that requires this subsystem.
     * @param name The name of the command.
     */
    protected CommandBuilder commandBuilder(String name) {
        return new CommandBuilder(name, this);
    }
}
