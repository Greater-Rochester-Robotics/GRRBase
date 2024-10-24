package org.team340.lib.dashboard;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import org.team340.lib.util.Alliance;

/**
 * Utility class for interfacing with GRRDashboard.
 */
public final class GRRDashboard {

    private GRRDashboard() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("GRRDashboard");
    private static final EventLoop periodic = new EventLoop();

    private static final BooleanSubscriber allianceOverrideActiveSub = nt
        .getBooleanTopic("AllianceOverride/active")
        .subscribe(false);
    private static final BooleanSubscriber allianceOverrideIsBlueSub = nt
        .getBooleanTopic("AllianceOverride/isBlue")
        .subscribe(false);

    private static final Map<String, Pair<Command, String>> autoOptions = new LinkedHashMap<>(); // { id: [command, json] }
    private static final StringArrayPublisher autoOptionsPub = nt.getStringArrayTopic("Autos/options").publish();
    private static final StringPublisher activeAutoPub = nt.getStringTopic("Autos/active").publish();
    private static final StringSubscriber selectedAutoSub;

    private static Command selectedAuto = Commands.none();

    static {
        String defaultAuto = addAuto("Do Nothing", selectedAuto);
        selectedAutoSub = nt.getStringTopic("Autos/selected").subscribe(defaultAuto);
        activeAutoPub.set(defaultAuto);
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param command The auto's command.
     */
    public static String addAuto(String label, Command command) {
        return addAuto(label, command, new Trajectory<SwerveSample>("", List.of(), List.of(), List.of()));
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param command The auto's command.
     * @param trajectories Trajectories utilized by the auto.
     */
    @SafeVarargs
    public static String addAuto(String label, Command command, Trajectory<SwerveSample>... trajectories) {
        String id = UUID.randomUUID().toString();
        List<BigDecimal[]> samples = new ArrayList<>();

        double time = 0.0;
        for (int i = 0; i < trajectories.length; i++) {
            for (SwerveSample sample : trajectories[i].sampleArray()) {
                samples.add(
                    new BigDecimal[] {
                        new BigDecimal(sample.x).setScale(3, RoundingMode.HALF_UP),
                        new BigDecimal(sample.y).setScale(3, RoundingMode.HALF_UP),
                        new BigDecimal(sample.heading).setScale(2, RoundingMode.HALF_UP),
                        new BigDecimal(sample.t + time).setScale(3, RoundingMode.HALF_UP)
                    }
                );
            }

            SwerveSample finalSample = trajectories[i].getFinalSample();
            if (finalSample != null) time += finalSample.t;
        }

        double t = time;
        String json = "";
        try {
            json = new ObjectMapper()
                .writeValueAsString(
                    new HashMap<>() {
                        {
                            put("id", id);
                            put("label", label);
                            put("samples", samples);
                            put("time", t);
                        }
                    }
                );
        } catch (Exception e) {
            e.printStackTrace();
            json = "";
        }

        if (json.isEmpty()) {
            json = "{ \"id\": \"" + id + "\", \"label\": \"" + label + "\", \"samples\": [], \"time\": 0 }";
        }

        autoOptions.put(id, Pair.of(command, json));
        autoOptionsPub.set(autoOptions.values().stream().map(entry -> entry.getSecond()).toArray(String[]::new));
        return id;
    }

    /**
     * Gets the command of the selected auto.
     */
    public static Command getSelectedAuto() {
        return selectedAuto;
    }

    /**
     * Syncs data with the dashboard. Must be called
     * periodically in order for this class to function.
     */
    public static void update() {
        if (allianceOverrideActiveSub.get()) {
            Alliance.enableOverride(allianceOverrideIsBlueSub.get());
        } else {
            Alliance.disableOverride();
        }

        for (String id : selectedAutoSub.readQueueValues()) {
            var entry = autoOptions.get(id);
            if (entry != null) {
                activeAutoPub.set(id);
                selectedAuto = entry.getFirst();
            }
        }

        periodic.poll();
    }

    /**
     * Binds an action to the dashboard's update loop.
     * @param action The action to bind.
     */
    static void bind(Runnable action) {
        periodic.bind(action);
    }
}
