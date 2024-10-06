package org.team340.lib.util.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import org.team340.lib.util.Sleep;

/**
 * Base class for configuring REV hardware.
 */
abstract class RevConfigBase<T> {

    private static final double CHECK_PERIOD = 0.015;

    private static final record RevConfigStep<T>(
        Function<T, REVLibError> applier,
        Function<T, Boolean> checker,
        boolean trustCheck,
        String name
    ) {}

    final List<RevConfigStep<T>> configSteps = new ArrayList<>();

    /**
     * Stores a configuration step.
     */
    void addStep(Function<T, REVLibError> applier, String name) {
        configSteps.add(new RevConfigStep<>(applier, device -> true, false, name));
    }

    /**
     * Stores a configuration step.
     */
    void addStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, String name) {
        configSteps.add(new RevConfigStep<>(applier, checker, true, name));
    }

    /**
     * Stores a configuration step.
     */
    void addStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, boolean trustCheck, String name) {
        configSteps.add(new RevConfigStep<T>(applier, checker, trustCheck, name));
    }

    /**
     * Applies the config. Note that this is a blocking operation. Errors
     * are printed when calling {@link RevConfigRegistry#burnFlashAll()}.
     * @param device The device to apply the config to.
     * @param identifier A string used to identify the device in logs.
     */
    void applySteps(T device, String identifier) {
        for (RevConfigStep<T> step : configSteps) {
            boolean ok = true;
            boolean check = true;
            String results = "";

            for (int i = 0; i < 3; i++) {
                REVLibError result = step.applier.apply(device);
                Sleep.seconds(CHECK_PERIOD, true);

                ok = REVLibError.kOk.equals(result) ||
                (RobotBase.isSimulation() && result.equals(REVLibError.kParamMismatchType));
                check = step.checker().apply(device);
                if (ok && check && step.trustCheck) break;

                results += (results.isEmpty() ? "" : ", ") + result.name() + (!check ? " (Failed Check)" : "");
            }

            if (!ok || !check) {
                RevConfigRegistry.addError(identifier + " \"" + step.name() + "\": " + results);
            }
        }
    }
}
