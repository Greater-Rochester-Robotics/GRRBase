package org.team340.lib.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Utility class for creating a trigger that is {@code true} when supplied triggers are all {@code false}.
 */
public final class TriggerLockout {

    private TriggerLockout() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Creates a trigger that returns {@code true} when supplied triggers are all {@code false}.
     * @param triggers The triggers to poll.
     */
    public static Trigger of(Trigger... triggers) {
        if (triggers.length == 0) return new Trigger(() -> true);
        Trigger trigger = triggers[0].negate();
        for (int i = 1; i < triggers.length; i++) {
            trigger = trigger.and(triggers[i].negate());
        }
        return trigger;
    }
}
