package org.team340.lib.logging.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;

@CustomLoggerFor(CANcoder.class)
public class CANcoderLogger extends ClassSpecificLogger<CANcoder> {

    private static final Map<CANcoder, Consumer<EpilogueBackend>> cache = new HashMap<>();

    public CANcoderLogger() {
        super(CANcoder.class);
    }

    @Override
    public void update(EpilogueBackend backend, CANcoder canCoder) {
        cache
            .computeIfAbsent(canCoder, key -> {
                var absolutePosition = canCoder.getAbsolutePosition(false);
                var magnetHealth = canCoder.getMagnetHealth(false);
                var position = canCoder.getPosition(false);
                var velocity = canCoder.getVelocity(false);

                BaseStatusSignal[] signals = { absolutePosition, magnetHealth, position, velocity };

                return b -> {
                    BaseStatusSignal.refreshAll(signals);
                    b.log("absolutePosition", absolutePosition.getValueAsDouble());
                    b.log("magnetHealth", magnetHealth.getValue().name());
                    b.log("position", position.getValueAsDouble());
                    b.log("velocity", velocity.getValueAsDouble());
                };
            })
            .accept(backend);
    }
}
