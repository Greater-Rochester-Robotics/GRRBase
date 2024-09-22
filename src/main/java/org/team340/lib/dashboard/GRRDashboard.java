package org.team340.lib.dashboard;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.RobotState;
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

    private static final BooleanPublisher robotEnabledPub = nt.getBooleanTopic("Robot/enabled").publish();
    private static final BooleanPublisher robotIsBluePub = nt.getBooleanTopic("Robot/isBlue").publish();
    private static final BooleanSubscriber allianceOverrideActiveSub = nt.getBooleanTopic("Robot/AllianceOverride/active").subscribe(false);
    private static final BooleanSubscriber allianceOverrideIsBlueSub = nt.getBooleanTopic("Robot/AllianceOverride/isBlue").subscribe(false);

    private static final Map<String, Pair<String, Command>> autoOptions = new LinkedHashMap<>(); // { id: [json, command] }
    private static final StringArrayPublisher autoOptionsPub = nt.getStringArrayTopic("Autos/options").publish();
    private static final StringPublisher activeAutoPub = nt.getStringTopic("Autos/active").publish();
    private static final StringSubscriber selectedAutoSub;

    private static final NetworkTable tunables = nt.getSubTable("Tunables");
    private static final List<StringPublisher> tunablePubs = new ArrayList<>();

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
        return addAuto(label, command, new ChoreoTrajectory[] {});
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param command The auto's command.
     * @param trajectory The trajectory utilized by the auto.
     */
    public static String addAuto(String label, Command command, ChoreoTrajectory trajectory) {
        return addAuto(label, command, new ChoreoTrajectory[] { trajectory });
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param command The auto's command.
     * @param trajectories Trajectories utilized by the auto.
     */
    public static String addAuto(String label, Command command, List<ChoreoTrajectory> trajectories) {
        return addAuto(label, command, trajectories.stream().toArray(ChoreoTrajectory[]::new));
    }

    /**
     * Adds an auto to the dashboard.
     * @param label The label for the auto.
     * @param command The auto's command.
     * @param trajectories Trajectories utilized by the auto.
     */
    public static String addAuto(String label, Command command, ChoreoTrajectory[] trajectories) {
        String id = UUID.randomUUID().toString();
        List<BigDecimal[]> points = new ArrayList<>();

        double lastTimestamp = 0.0;
        for (int i = 0; i < trajectories.length; i++) {
            ChoreoTrajectoryState[] states = trajectories[i].getStates();
            if (i > 0 && trajectories[i - 1].getStates().length > 0) lastTimestamp += trajectories[i - 1].getFinalState().timestamp;
            for (ChoreoTrajectoryState state : states) {
                points.add(
                    new BigDecimal[] {
                        new BigDecimal(state.x).setScale(3, RoundingMode.HALF_UP),
                        new BigDecimal(state.y).setScale(3, RoundingMode.HALF_UP),
                        new BigDecimal(state.heading).setScale(2, RoundingMode.HALF_UP),
                        new BigDecimal(state.timestamp + lastTimestamp).setScale(3, RoundingMode.HALF_UP),
                    }
                );
            }
        }

        ChoreoTrajectory lastTrajectory = trajectories.length > 0 ? trajectories[trajectories.length - 1] : new ChoreoTrajectory();
        double time = lastTimestamp + (lastTrajectory.getStates().length > 0 ? lastTrajectory.getFinalState().timestamp : 0.0);

        String json = "";
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
        autoOptions.put(id, Pair.of(json, command));
        autoOptionsPub.set(autoOptions.values().stream().map(entry -> entry.getFirst()).toArray(String[]::new));
        return id;
    }

    /**
     * Gets the command of the selected auto.
     */
    public static Command getSelectedAuto() {
        return selectedAuto;
    }

    /**
     * Adds a {@link Tunable} to the dashboard.
     * @param tunable The {@link Tunable} to add.
     */
    public static void addTunable(Tunable tunable) {
        List<HashMap<String, ?>> fields = new ArrayList<>();
        for (var field : tunable.fields) {
            periodic.bind(field.init().apply(tunables));
            fields.add(
                new HashMap<>() {
                    {
                        put("name", field.name());
                        put("type", field.type().getValueStr());
                        put("default", field.defaultValue());
                    }
                }
            );
        }

        String json = "";
        try {
            json =
                new ObjectMapper()
                    .writeValueAsString(
                        new HashMap<>() {
                            {
                                put("devOnly", tunable.devOnly);
                                put("fields", fields);
                            }
                        }
                    );
        } catch (Exception e) {
            e.printStackTrace();
            json = "";
        }

        if (json.isEmpty()) return;
        StringPublisher pub = tunables.getStringTopic(tunable.name).publish();
        tunablePubs.add(pub);
        pub.set(json);
    }

    /**
     * Syncs data with the dashboard. Must be called
     * periodically in order for this class to function.
     */
    public static void update() {
        robotEnabledPub.set(RobotState.isEnabled());

        if (allianceOverrideActiveSub.get()) {
            Alliance.enableOverride(allianceOverrideIsBlueSub.get());
        } else {
            Alliance.disableOverride();
        }

        robotIsBluePub.set(Alliance.isBlue());

        for (String id : selectedAutoSub.readQueueValues()) {
            var entry = autoOptions.get(id);
            if (entry != null) {
                activeAutoPub.set(id);
                selectedAuto = entry.getSecond();
            }
        }

        periodic.poll();
    }
}
