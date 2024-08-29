package org.team340.lib;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.concurrent.locks.ReentrantLock;

/**
 * Utility class for interfacing with the dashboard.
 */
public final class GRRDashboard {

    /**
     * A sendable for the auto chooser.
     */
    private static class AutoChooserSendable implements Sendable {

        private final ReentrantLock selectedMutex = new ReentrantLock();

        private String defaultChoice;
        private String selected;
        private Command choice = Commands.none();
        private Map<String, Map.Entry<String, Command>> options = new LinkedHashMap<>(); // { [id]: [json, command] }

        /**
         * Creates the auto chooser. Automatically adds a default option that does nothing.
         */
        public AutoChooserSendable() {
            defaultChoice = addOption("Do Nothing", new ChoreoTrajectory[] { new ChoreoTrajectory() }, Commands.none());
        }

        /**
         * Gets the command of the selected auto.
         */
        public Command getSelected() {
            return choice;
        }

        /**
         * Adds an option.
         * @param label The option's label.
         * @param trajectories Trajectories used by the auto.
         * @param command The option's command.
         * @return The option's ID.
         */
        public String addOption(String label, ChoreoTrajectory[] trajectories, Command command) {
            String id = UUID.randomUUID().toString();
            String json = "";

            List<double[]> points = new ArrayList<>();
            double lastTimestamp = 0.0;

            for (int i = 0; i < trajectories.length; i++) {
                ChoreoTrajectoryState[] states = trajectories[i].getStates();
                if (i > 0 && trajectories[i - 1].getStates().length > 0) lastTimestamp += trajectories[i - 1].getFinalState().timestamp;
                for (ChoreoTrajectoryState state : states) {
                    points.add(new double[] { state.x, state.y, state.heading, state.timestamp + lastTimestamp });
                }
            }

            ChoreoTrajectory lastTrajectory = trajectories.length > 0 ? trajectories[trajectories.length - 1] : new ChoreoTrajectory();
            double time = lastTimestamp + (lastTrajectory.getStates().length > 0 ? lastTrajectory.getFinalState().timestamp : 0.0);

            try {
                json =
                    new ObjectMapper()
                        .writeValueAsString(
                            new HashMap<>() {
                                {
                                    put("id", id);
                                    put("label", label);
                                    put("points", points);
                                    put("time", time);
                                }
                            }
                        );
            } catch (Exception e) {
                e.printStackTrace();
                json = "";
            }

            if (json.isEmpty()) json = "{ \"id\": \"" + id + "\", \"label\": \"" + label + "\", \"points\": [] }";
            options.put(id, Map.entry(json, command));
            return id;
        }

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addStringProperty("default", () -> defaultChoice, null);
            builder.addStringArrayProperty(
                "options",
                () -> options.values().stream().map(entry -> entry.getKey()).toArray(String[]::new),
                null
            );
            builder.addStringProperty(
                "active",
                () -> {
                    selectedMutex.lock();
                    try {
                        if (selected != null) return selected; else return defaultChoice;
                    } finally {
                        selectedMutex.unlock();
                    }
                },
                null
            );
            builder.addStringProperty(
                "selected",
                null,
                (String value) -> {
                    selectedMutex.lock();
                    try {
                        Map.Entry<String, Command> entry = options.get(value);
                        if (entry != null) {
                            selected = value;
                            choice = entry.getValue();
                        }
                    } finally {
                        selectedMutex.unlock();
                    }
                }
            );
        }
    }
}
