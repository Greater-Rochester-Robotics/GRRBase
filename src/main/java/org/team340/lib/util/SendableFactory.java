package org.team340.lib.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.Consumer;

/**
 * Factory for creating {@link Sendable}s inline.
 */
public class SendableFactory {

    private SendableFactory() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Creates a {@link Sendable} object.
     * @param initSendable A {@link SendableBuilder} consumer, called when the return object's {@code initSendable()} method is invoked.
     * @return The created sendable.
     */
    public static Sendable create(Consumer<SendableBuilder> initSendable) {
        return new SendableImpl(initSendable);
    }

    private static class SendableImpl implements Sendable {

        private final Consumer<SendableBuilder> initSendableConsumer;

        public SendableImpl(Consumer<SendableBuilder> initSendable) {
            initSendableConsumer = initSendable;
        }

        public void initSendable(SendableBuilder builder) {
            initSendableConsumer.accept(builder);
        }
    }
}
