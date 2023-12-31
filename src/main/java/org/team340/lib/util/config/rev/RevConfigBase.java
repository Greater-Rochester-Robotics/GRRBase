package org.team340.lib.util.config.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * Base class for configuring REV hardware.
 */
abstract class RevConfigBase<T> {

    private static final record RevConfigStep<T>(
        Function<T, REVLibError> applier,
        Function<T, Boolean> checker,
        boolean trustCheck,
        String name
    ) {
        public RevConfigStep(Function<T, REVLibError> applier, String name) {
            this(applier, hardware -> true, false, name);
        }

        public RevConfigStep(Function<T, REVLibError> applier, Function<T, Boolean> checker, String name) {
            this(applier, checker, true, name);
        }
    }

    private final List<RevConfigStep<T>> configSteps = new ArrayList<>();

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
     * Applies the config.
     * @param hardware The hardware to apply the config to.
     * @param identifier A string used to identify the hardware in logs.
     */
    void applySteps(T hardware, String identifier) {
        for (RevConfigStep<T> step : configSteps) {
            applyStep(hardware, identifier, step, new String[RevConfigUtils.SET_ITERATIONS], RevConfigUtils.SET_ITERATIONS);
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
            if (!RobotBase.isSimulation()) {
                try {
                    Thread.sleep((long) RevConfigUtils.CHECK_SLEEP);
                } catch (Exception e) {}
            }

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
            boolean hadFailure = false;
            for (int i = 0; i < results.length; i++) {
                if (
                    !(results[i].equals(REVLibError.kOk.name()) || results[i].equals("Skipped")) &&
                    !(RobotBase.isSimulation() && results[i].startsWith(REVLibError.kParamMismatchType.name()))
                ) hadFailure = true;
                resultsString += results[i];
                if (i != results.length - 1) resultsString += ", ";
            }

            if (hadFailure) {
                DriverStation.reportWarning(
                    "Failure(s) encountered while configuring \"" + step.name() + "\" on " + identifier + ": " + resultsString,
                    true
                );
            } else {
                RevConfigUtils.addSuccess(identifier + " \"" + step.name() + "\": " + resultsString);
            }
        } else {
            applyStep(hardware, identifier, step, results, iterationsLeft);
        }
    }
}
