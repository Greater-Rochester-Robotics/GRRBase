import { derived } from "svelte/store";
import { DEFAULT_URI } from "./constants";
import { NTSvelte } from "./lib/NTSvelte";

export const nt = new NTSvelte(`GRRDashboard`, DEFAULT_URI, 40);

export const NTURI = nt.getURIStore();
export const NTConnected = nt.getConnectionStore();
export const NTBitrate = nt.getBitrateStore();
export const NTLatency = nt.getLatencyStore();
export const NTTopicMap = nt.getTopicMapStore();
export const NTTopicMapHierarchy = nt.getTopicMapHierarchyStore();

export const RobotEnabled = nt.createSubscription<boolean>(`/GRRDashboard/Robot/enabled`, false);
export const RobotMatchTime = nt.createSubscription<number>(`/GRRDashboard/Robot/matchTime`, 0);
export const RobotBlueAlliance = nt.createSubscription<boolean>(`/GRRDashboard/Robot/blueAlliance`, true);
export const RobotVoltage = nt.createSubscription<number>(`/GRRDashboard/Robot/voltage`, 0);

export const AutosActive = nt.createSubscription<string>(`/GRRDashboard/Autos/active`, ``);
export const AutosOptions = nt.createSubscription<string[]>(`/GRRDashboard/Autos/options`, []);
export const AutosSelected = nt.createPublisher<string>(`/GRRDashboard/Autos/selected`, `string`, ``);

export const FieldRobot = nt.createSubscription<[number, number, number]>(`/GRRDashboard/Subsystems/Swerve/Field/Robot`, [0, 0, 0]);
export const FieldModules = derived(
    nt.createSubscription<number[]>(`/GRRDashboard/Subsystems/Swerve/Field/Modules`, new Array(12).fill(-1)),
    ($v) => {
        return $v?.map((_, i) => (i % 3 === 0 ? $v.slice(i, i + 3) : null)).filter((v) => v) ?? ([] as number[][]);
    },
);

nt.connect();
