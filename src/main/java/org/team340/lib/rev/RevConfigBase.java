package org.team340.lib.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
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
    private static final int DEFAULT_ITERATIONS = 3;

    private static final record RevConfigStep<T>(
        Function<T, REVLibError> applier,
        Function<T, Boolean> checker,
        boolean trustCheck,
        int setIterations,
        String name
    ) {
        public RevConfigStep(Function<T, REVLibError> applier, String name) {
            this(applier, hardware -> true, false, name);
        }

        public RevConfigStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, String name) {
            this(applier, checker, true, name);
        }

        public RevConfigStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, boolean trustCheck, String name) {
            this(applier, checker, trustCheck, DEFAULT_ITERATIONS, name);
        }
    }

    final List<RevConfigStep<T>> configSteps = new ArrayList<>();

    /**
     * Creates a base config with no config steps.
     */
    RevConfigBase() {}

    /**
     * Creates a base config that copies the config steps from the base provided.
     * @param base The config to copy the steps from.
     */
    RevConfigBase(RevConfigBase<T> base) {
        for (RevConfigStep<T> step : base.configSteps) {
            configSteps.add(step);
        }
    }

    /**
     * Stores a configuration step.
     */
    void addStep(Function<T, REVLibError> applier, String name) {
        configSteps.add(new RevConfigStep<>(applier, name));
    }

    /**
     * Stores a configuration step.
     */
    void addStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, String name) {
        configSteps.add(new RevConfigStep<>(applier, checker, name));
    }

    /**
     * Stores a configuration step.
     */
    void addStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, boolean trustCheck, String name) {
        configSteps.add(new RevConfigStep<T>(applier, checker, trustCheck, name));
    }

    /**
     * Stores a configuration step.
     */
    void addStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, boolean trustCheck, int setIterations, String name) {
        configSteps.add(new RevConfigStep<T>(applier, checker, trustCheck, setIterations, name));
    }

    /**
     * Applies the config. Note that this is a blocking operation.
     * @param hardware The hardware to apply the config to.
     * @param identifier A string used to identify the hardware in logs.
     */
    void applySteps(T hardware, String identifier) {
        for (RevConfigStep<T> step : configSteps) {
            applyStep(hardware, identifier, step, new String[DEFAULT_ITERATIONS], DEFAULT_ITERATIONS);
        }
    }

    /**
     * Applies a config step.
     * @param hardware The hardware to apply to.
     * @param identifier A string used to identify the hardware in logs.
     * @param step The step to apply.
     * @param results A mutable array to push results to.
     * @param iterationsLeft The number of iterations left before failing.
     */
    private void applyStep(T hardware, String identifier, RevConfigStep<T> step, String[] results, int iterationsLeft) {
        try {
            REVLibError status = step.applier().apply(hardware);
            Sleep.seconds(CHECK_PERIOD);

            boolean check = step.checker().apply(hardware);
            results[results.length - iterationsLeft] = status.name() + (!check ? " (Failed Check)" : "");

            if (step.trustCheck() && check && REVLibError.kOk.equals(status)) {
                for (int i = 0; i < results.length; i++) {
                    if (i <= results.length - iterationsLeft) continue;
                    results[i] = "Skipped";
                }
                iterationsLeft = 1;
            }
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), true);
            results[results.length - iterationsLeft] = e.getClass().getSimpleName();
        }

        iterationsLeft--;
        if (iterationsLeft <= 0) {
            String resultsString = "";
            boolean hadSuccess = false;
            for (int i = 0; i < results.length; i++) {
                if (
                    results[i].equals(REVLibError.kOk.name()) ||
                    results[i].equals("Skipped") ||
                    (RobotBase.isSimulation() && results[i].startsWith(REVLibError.kParamMismatchType.name()))
                ) hadSuccess = true;
                resultsString += results[i];
                if (i != results.length - 1) resultsString += ", ";
            }

            if (!hadSuccess) {
                RevConfigRegistry.addError(identifier + " \"" + step.name() + "\": " + resultsString);
            }
        } else {
            applyStep(hardware, identifier, step, results, iterationsLeft);
        }
    }
}
