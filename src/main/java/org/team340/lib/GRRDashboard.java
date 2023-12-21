package org.team340.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.team340.lib.drivers.controller.Controller2;
import org.team340.lib.util.Math2;

/**
 * Utility class for interfacing with the dashboard.
 */
public final class GRRDashboard {

    private static final ReentrantLock localTableMutex = new ReentrantLock();
    private static final Watchdog watchdog = new Watchdog(Double.MAX_VALUE, GRRDashboard::reportOverrun);
    private static final List<Supplier<Double>> powerUsageGetters = new ArrayList<>();
    private static final List<Runnable> powerUsageUpdaters = new ArrayList<>();
    private static double lastPowerUsage = 0.0;
    private static double lastWatchdog = 0.0;

    private GRRDashboard() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * The network table used by GRRDashboard.
     */
    private static final NetworkTable nt = NetworkTableInstance.getDefault().getTable("GRRDashboard");
    /**
     * A local table assigning values to keys in network tables.
     */
    private static final Map<String, Sendable> localTable = new HashMap<>();
    /**
     * The auto chooser.
     */
    private static final SendableChooser<Command> autoChooser = new SendableChooser<>();
    /**
     * The notifier that runs telemetry updates on a separate thread.
     */
    private static Notifier notifier;
    /**
     * If the dashboard has been initialized.
     */
    private static boolean initialized = false;

    /**
     * Initializes the dashboard, with telemetry polling and updating performed on the main thread.
     * Power usage updates are also performed on the main thread.
     * @param robot The robot base.
     * @param telemetryPeriod The period in seconds to update telemetry at.
     * @param powerUsagePeriod The period in seconds to update power usage at.
     */
    public static void initSync(TimedRobot robot, double telemetryPeriod, double powerUsagePeriod) {
        init(robot, telemetryPeriod, powerUsagePeriod);
        robot.addPeriodic(GRRDashboard::updateValues, telemetryPeriod);
    }

    /**
     * Initializes the dashboard, with telemetry polling and updating performed on a separate thread.
     * Power usage updates are still performed on the main thread.
     * @param robot The robot base.
     * @param telemetryPeriod The period in seconds to update telemetry at.
     * @param powerUsagePeriod The period in seconds to update power usage at.
     */
    public static void initAsync(TimedRobot robot, double telemetryPeriod, double powerUsagePeriod) {
        init(robot, telemetryPeriod, powerUsagePeriod);
        notifier = new Notifier(GRRDashboard::updateValues);
        notifier.setName("GRRDashboard");
        notifier.startPeriodic(telemetryPeriod);
    }

    /**
     * Initializes all parts of the dashboard except for updating telemetry.
     */
    private static synchronized void init(TimedRobot robot, double telemetryPeriod, double powerUsagePeriod) {
        if (initialized) throw new IllegalStateException("GRRDashboard is already initialized");
        initialized = true;

        publish("Robot", new RobotSendable());

        autoChooser.setDefaultOption(serializeAuto("Do Nothing"), Commands.none().withName("Do Nothing"));
        publish("Autos", autoChooser);

        publish("SystemsCheck", Commands.none().withName("SystemsCheck"));

        watchdog.setTimeout(telemetryPeriod);
        robot.addPeriodic(GRRDashboard::updatePowerUsage, powerUsagePeriod);
    }

    /**
     * Publishes a sendable to network tables.
     * @param key The key to publish under.
     * @param sendable The sendable.
     */
    public static void publish(String key, Sendable sendable) {
        if (sendable == null) return;
        try {
            localTableMutex.lock();
            Sendable existing = localTable.get(key);
            if (existing == null || !existing.equals(sendable)) {
                localTable.put(key, sendable);
                NetworkTable table = nt.getSubTable(key);
                SendableBuilderImpl builder = new SendableBuilderImpl();
                builder.setTable(table);
                SendableRegistry.publish(sendable, builder);
                builder.startListeners();
            }
        } finally {
            localTableMutex.unlock();
        }
    }

    /**
     * Gets the currently selected auto command.
     */
    public static Command getAutoCommand() {
        Command selected = autoChooser.getSelected();
        return selected != null ? selected : Commands.none().withName("Do Nothing");
    }

    /**
     * Sets the systems check command.
     * @param command The systems check command.
     */
    public static void setSystemsCheck(Command command) {
        publish("SystemsCheck", command.withName("SystemsCheck"));
    }

    /**
     * Adds a command to the dashboard.
     * @param label The label for the command.
     * @param command The command to add to the dashboard.
     */
    public static void addCommand(String label, Command command) {
        publish("Commands/" + label, command.withName(label));
    }

    /**
     * Adds an auto command to the dashboard.
     * @param label The label for the command.
     * @param command The command to add to the dashboard.
     */
    public static void addAutoCommand(String label, Command command) {
        addAutoCommand(label, command, "");
    }

    /**
     * Adds an auto command to the dashboard.
     * @param label The label for the command.
     * @param command The command to add to the dashboard.
     * @param trajFile The name of trajectory file generated by Choreo used by the command.
     */
    public static void addAutoCommand(String label, Command command, String trajFile) {
        autoChooser.addOption(serializeAuto(label), command.withName(label));
    }

    /**
     * Adds a controller.
     * @param label The label for the controller.
     * @param controller The controller.
     */
    public static void addController(Controller2 controller) {
        publish("Controllers/" + controller.getLabel(), controller);
    }

    /**
     * Adds a subsystem to the dashboard.
     * @param label The label for the subsystem.
     */
    public static void addSubsystem(GRRSubsystem subsystem) {
        addSubsystemSendable("Details", subsystem, subsystem);
    }

    /**
     * Adds a sendable property to a subsystem's directory in NT.
     * @param key The key to assign the sendable to.
     * @param subsystem The subsystem the sendable is associated with.
     * @param sendable The Sendable.
     */
    public static void addSubsystemSendable(String key, GRRSubsystem subsystem, Sendable sendable) {
        publish("Subsystems/" + subsystem.getName() + "/" + key, sendable);
    }

    /**
     * Adds hardware to the dashboard.
     * Done automatically when using hardware factories in {@link GRRSubsystem}.
     * @param subsystem The subsystem the hardware is associated with.
     * @param hardware A sendable to represent the hardware.
     */
    static void addHardware(GRRSubsystem subsystem, HardwareSendables.Hardware hardware) {
        addSubsystemSendable("Hardware/" + hardware.getKey(), subsystem, hardware);
    }

    /**
     * Adds hardware to the dashboard.
     * Done automatically when using hardware factories in {@link GRRSubsystem}.
     * @param subsystem The subsystem the hardware is associated with.
     * @param hardware A sendable to represent the hardware.
     */
    static void addHardware(GRRSubsystem subsystem, HardwareSendables.PoweredHardware hardware) {
        addSubsystemSendable("Hardware/" + hardware.getKey(), subsystem, hardware);
        powerUsageGetters.add(hardware::getPowerUsage);
        powerUsageUpdaters.add(hardware::updatePowerUsage);
    }

    /**
     * Updates sendable data.
     */
    private static void updateValues() {
        watchdog.reset();

        try {
            localTableMutex.lock();
            for (Map.Entry<String, Sendable> entry : localTable.entrySet()) {
                SendableRegistry.update(entry.getValue());
                watchdog.addEpoch(entry.getKey());
            }
        } finally {
            localTableMutex.unlock();
        }

        watchdog.disable();
        lastWatchdog = watchdog.getTime();
        if (watchdog.isExpired()) watchdog.printEpochs();
    }

    private static void updatePowerUsage() {
        for (Runnable updater : powerUsageUpdaters) updater.run();

        double usageTotal = 0.0;
        for (Supplier<Double> getter : powerUsageGetters) usageTotal += getter.get();
        lastPowerUsage = usageTotal;
    }

    /**
     * Report an overrun.
     */
    private static void reportOverrun() {
        DriverStation.reportWarning(
            "Dashboard update frequency of " + (watchdog != null ? watchdog.getTimeout() : 0) + "s overrun\n",
            false
        );
    }

    /**
     * Serializes an auto into JSON.
     * @param label The label for the command.
     */
    private static String serializeAuto(String label) {
        // TODO

        // List<Map<String, JSONObject>> parsedSplines = new ArrayList<>();

        // if (!pathFile.isEmpty()) {
        //     try {
        //         JSONArray rawWaypoints = (JSONArray) (
        //             (JSONObject) new JSONParser()
        //                 .parse(new FileReader(new File(Filesystem.getDeployDirectory(), "choreo/" + trajFile)))
        //         ).get("waypoints");

        //         if (rawWaypoints != null) {
        //             for (int i = 0; i < rawWaypoints.size(); i++) {
        //                 JSONObject rawWaypoint = (JSONObject) rawWaypoints.get(i);
        //                 JSONObject nextRawWaypoint = i < rawWaypoints.size() - 1 ? (JSONObject) rawWaypoints.get(i + 1) : null;
        //                 if (rawWaypoint == null) continue;

        //                 parsedSplines.add(
        //                     new HashMap<>() {
        //                         {
        //                             put("p0", (JSONObject) rawWaypoint.get("anchorPoint"));
        //                             put("p1", (JSONObject) rawWaypoint.get("nextControl"));
        //                             put("p2", nextRawWaypoint != null ? (JSONObject) nextRawWaypoint.get("prevControl") : null);
        //                             put("p3", nextRawWaypoint != null ? (JSONObject) nextRawWaypoint.get("anchorPoint") : null);
        //                         }
        //                     }
        //                 );
        //             }
        //         }
        //     } catch (Exception e) {
        //         e.printStackTrace();
        //     }
        // }

        return "{ \"label\": " + label + ", \"splines\": [] }";
    }

    /**
     * A sendable that represents the robot.
     */
    private static class RobotSendable implements Sendable {

        public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("blueAlliance", () -> DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Blue), null);
            builder.addDoubleProperty("cpuTemperature", () -> Math2.toFixed(RobotController.getCPUTemp()), null);
            builder.addBooleanProperty("enabled", DriverStation::isEnabled, null);
            builder.addIntegerProperty("matchTime", () -> (int) DriverStation.getMatchTime(), null);
            builder.addDoubleProperty("ntUpdateTime", () -> Math2.toFixed(lastWatchdog, 1e-6), null);
            builder.addDoubleProperty("powerUsage", () -> Math2.toFixed(lastPowerUsage), null);
            builder.addIntegerProperty("timestamp", RobotController::getFPGATime, null);
            builder.addDoubleProperty("voltage", () -> Math2.toFixed(RobotController.getBatteryVoltage()), null);
        }
    }
}
