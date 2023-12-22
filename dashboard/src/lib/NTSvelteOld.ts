import { readable, writable, type Readable, type Writable, derived } from "svelte/store";
import { MessagePack } from "./MessagePack";

export type NTTopicMapNestedValue = string | NTTopicMapNested;
export interface NTTopicMapNested extends Record<string, string | NTTopicMapNestedValue> {}

/**
 * Types supported by Network Tables.
 */
export type NTType = boolean | number | string | Uint8Array | boolean[] | number[] | string[];

/**
 * Network tables type codes.
 */
export const NTTypeCodes = {
    boolean: 0,
    double: 1,
    int: 2,
    float: 3,
    string: 4,
    json: 4,
    raw: 5,
    rpc: 5,
    msgpack: 5,
    protobuf: 5,
    "boolean[]": 16,
    "double[]": 17,
    "int[]": 18,
    "float[]": 19,
    "string[]": 20,
} as const;

export type NTTypeString = keyof typeof NTTypeCodes;
export type NTReference = [string, NTTypeString];
export type NTHierarchyValue = NTReference | NTHierarchy;
export interface NTHierarchy extends Record<string, NTHierarchyValue> {}

/**
 * A topic in network tables.
 */
export class NTTopic {
    private readonly _key: string;
    private _id: number | null;
    private _type: NTTypeString | null;
    private _value: NTType | null = null;
    private _valueTimestamp: number | null = null;
    private _valueHistory: Map<number, NTType | null> = new Map();
    private _listeners: Set<(value: NTType | null) => void> = new Set();
    private _pubuid: number | null = null;

    /**
     * Create a topic.
     * @param key
     * @param id
     */
    constructor(key: string, id: number | null = null, type: NTTypeString | null = null) {
        this._key = key;
        this._id = id;
        this._type = type;
    }

    public getKey(): string {
        return this._key;
    }

    public setId(id: number | null): void {
        this._id = id;
    }

    public getId(): number | null {
        return this._id;
    }

    public setType(type: NTTypeString | null): void {
        this._type = type;
    }

    public getType(): NTTypeString | null {
        return this._type;
    }

    public setValue(value: NTType | null, timestamp: number | null = null): void {
        if (timestamp === null || timestamp > (this._valueTimestamp ?? 0)) {
            this._value = value;
            this._valueTimestamp = timestamp;
            this._listeners.forEach((listener) => listener(value));
        }

        if (timestamp !== null) {
            this._valueHistory.set(timestamp, value);
        }
    }

    public getValue(): NTType | null {
        return this._value;
    }

    public getValueHistory(): Map<number, NTType | null> {
        return this._valueHistory;
    }

    public addListener(listener: (value: NTType | null) => void): void {
        this._listeners.add(listener);
        listener(this._value);
    }

    public removeListener(listener: (value: NTType | null) => void): void {
        this._listeners.delete(listener);
    }

    public setPublished(pubuid: number): void {
        this._pubuid = pubuid;
    }

    public setUnpublished(): void {
        this._pubuid = null;
    }

    public isPublished(): boolean {
        return this._pubuid !== null;
    }

    public getPubuid(): number | null {
        return this._pubuid;
    }
}

/**
 * A NetworkTables 4 implementation for svelte stores.
 * Automatically subscribes to all topics.
 */
export class NTSvelte {
    private static readonly TIMESTAMP_INTERVAL = 2500;

    private readonly _appName: string;
    private readonly _updateInterval: number;

    private _bitrateStore: Writable<string> = writable(`0.00`);
    private _byteLengthCounter: number = 0;
    private _connectionStore: Writable<boolean> = writable(false);
    private _lastServerTime: number | null = null;
    private _latencyStore: Writable<string> = writable(`0.00`);
    private _serverOffset: number | null = null;
    private _serverTime: number | null = null;
    private _started = false;
    private _subuidNonce = 0;
    private _topics: Map<string, NTTopic> = new Map();
    private _topicMap: Writable<NTReference[]> = writable([]);
    private _uri: string;
    private _uriStore: Writable<string>;
    private _ws: WebSocket | null = null;
    private _wsListeners: Record<keyof WebSocketEventMap, Function> = {
        open: this._onOpen.bind(this),
        close: this._onClose.bind(this),
        error: this._onError.bind(this),
        message: this._onMessage.bind(this),
    };

    /**
     * Create the NT client.
     * @param appName The app name.
     * @param uri The initial URI to connect to.
     * @param updateInterval The interval at which the client should receive values from network tables at.
     */
    constructor(appName: string, uri: string, updateInterval: number) {
        this._appName = appName;
        this._uri = uri;
        this._uriStore = writable(uri);
        this._updateInterval = updateInterval;

        setInterval(() => {
            if (this._ws?.readyState === WebSocket.OPEN) {
                if (this._lastServerTime !== null && this._lastServerTime === this._serverTime) {
                    this.disconnect(`Timed Out`);
                    return;
                } else {
                    this._lastServerTime = this._serverTime;
                }

                this._sendTimestamp();

                const bitrate = ((this._byteLengthCounter * 0.0016) / (NTSvelte.TIMESTAMP_INTERVAL / 1000)).toFixed(2);
                this._bitrateStore.set(bitrate);
                console.log(`[NT] Bitrate: ${bitrate} kb/s`);
                this._byteLengthCounter = 0;
            }
        }, NTSvelte.TIMESTAMP_INTERVAL);
    }

    /**
     * Connect to the robot.
     */
    public connect(): void {
        if (!this._started) {
            this._started = true;
            this._connect();
        }
    }

    /**
     * Disconnect from the robot.
     */
    public disconnect(reason: string = `Client Disconnect`): void {
        if (this._started) {
            this._started = false;
            this._ws?.close(1000, reason);
            this._ws = null;
            console.log(`[NT] Disconnected: ${reason}`);
        }
    }

    public setURI(uri: string): void {
        this.disconnect();
        this._uri = uri;
        this._uriStore.set(uri);
        this.connect();
    }

    public getURIStore(): Readable<string> {
        return this._uriStore;
    }

    public getConnectionStore(): Readable<boolean> {
        return this._connectionStore;
    }

    public getBitrateStore(): Readable<string> {
        return this._bitrateStore;
    }

    public getLatencyStore(): Readable<string> {
        return this._latencyStore;
    }

    public getTopicMapStore(): Readable<NTReference[]> {
        return this._topicMap;
    }

    public getTopicMapHierarchyStore(): Readable<NTHierarchy> {
        return derived(this._topicMap, (value) => {
            return value.reduce((p, c) => {
                let current = p;
                c[0].split(`/`).forEach((topic, i, array) => {
                    if (i + 1 === array.length) current[topic] = [c[0], c[1]];
                    else {
                        const n = typeof current[topic] === `object` ? (current[topic] as NTHierarchy) : {};
                        current[topic] = n;
                        current = n;
                    }
                });
                return p;
            }, {} as NTHierarchy);
        });
    }

    getTopicHistoryMap<T extends NTType>(key: string): Map<number, T | null> {
        return this._topics.get(key)?.getValueHistory() ?? new Map();
    }

    public createSubscription<T extends NTType>(key: string, defaultValue: T | null): Readable<T | null> {
        return readable(defaultValue, (set) => {
            const listener = (value: T | null) => {
                set(value ?? defaultValue);
            };

            const existing = this._topics.get(key);
            if (existing) {
                existing.addListener(listener as any);
            } else {
                const newTopic = new NTTopic(key);
                newTopic.addListener(listener as any);
                this._topics.set(key, newTopic);
            }

            return () => {
                const existing = this._topics.get(key);
                if (existing) {
                    existing.removeListener(listener as any);
                }
            };
        });
    }

    public createPublisher<T extends NTType>(key: string, type: keyof typeof NTTypeCodes, firstValue: T): Writable<T | null> {
        let topic: NTTopic;
        const existing = this._topics.get(key);
        if (existing) topic = existing;
        else {
            topic = new NTTopic(key, null, type);
            this._topics.set(key, topic);
            this._topicMap.update((last) => [...last, [key, type]]);
        }

        const inst = writable<T | null>(firstValue, (set) => {
            const listener = (value: T | null) => {
                set(value);
            };

            topic.addListener(listener as any);

            return () => {
                const existing = this._topics.get(key);
                if (existing) {
                    existing.removeListener(listener as any);
                }
            };
        });

        return {
            subscribe: inst.subscribe,
            set: (value: T | null) => {
                if (value === null) {
                    if (topic.getPubuid()) {
                        this._sendJSON(`unpublish`, { pubuid: topic.getPubuid() });
                    }
                    topic.setUnpublished();
                } else {
                    if (!topic.isPublished()) {
                        const pubuid = this._getPubuid();
                        this._sendJSON(`publish`, {
                            name: key,
                            pubuid: pubuid,
                            type,
                        });

                        topic.setPublished(pubuid);
                    }

                    this._sendMessagePack([
                        topic.getPubuid()!,
                        (this._serverOffset ?? 0) + Date.now() * 1000,
                        NTTypeCodes[type],
                        value as any,
                    ]);
                }

                topic.setValue(value, this._serverOffset !== null ? this._serverOffset + Date.now() * 1000 : null);
            },
            update: inst.update,
        };
    }

    /**
     * Connect to the robot.
     */
    private _connect(): void {
        const url = encodeURI(`ws://${this._uri}:5810/nt/${this._appName}`);
        this._ws ??= new WebSocket(url, `networktables.first.wpi.edu`);
        this._ws.binaryType = `arraybuffer`;

        console.log(`[NT] Connecting to ${url}`);

        for (const [type, listener] of Object.entries(this._wsListeners)) {
            this._ws.addEventListener(type, listener as any);
        }
    }

    /**
     * Send the client's current time.
     */
    private _sendTimestamp(): void {
        this._sendMessagePack([-1, 0, NTTypeCodes.int, Date.now() * 1000]);
    }

    /**
     * Send MessagePack binary data.
     * @param data The data to send.
     * @returns If the data was sent.
     */
    private _sendMessagePack(data: number[]): boolean {
        if (this._ws?.readyState === WebSocket.OPEN) {
            const msg = MessagePack.serialize(data);
            this._ws.send(msg);
            this._byteLengthCounter += msg.byteLength;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Send JSON data.
     * @param method The method value to send.
     * @param params JSON parameters.
     * @returns If the data was sent.
     */
    private _sendJSON(method: string, params: any): boolean {
        if (this._ws?.readyState === WebSocket.OPEN) {
            const msg = JSON.stringify([{ method, params }]);
            this._ws.send(msg);
            this._byteLengthCounter += new Blob([msg]).size;
            return true;
        } else {
            return false;
        }
    }

    private _onOpen(): void {
        console.log(`[NT] Connected`);
        this._sendTimestamp();
        this._sendJSON(`subscribe`, {
            topics: [`/`],
            subuid: this._getSubuid(),
            options: {
                all: true,
                periodic: this._updateInterval / 1000,
                prefix: true,
            },
        });

        setTimeout(() => {
            if (this._ws?.readyState === WebSocket.OPEN) {
                this._connectionStore.set(true);
            }
        }, 100);
    }

    private _onClose(event: CloseEvent): void {
        for (const [type, listener] of Object.entries(this._wsListeners)) {
            this._ws?.removeEventListener(type, listener as any);
        }

        this._ws = null;
        this._bitrateStore.set(`0.00`);
        this._byteLengthCounter = 0;
        this._connectionStore.set(false);
        this._lastServerTime = null;
        this._latencyStore.set(`0.00`);
        this._serverTime = null;
        this._topics.forEach((topic) => {
            if (topic.isPublished()) {
                topic.setUnpublished();
            } else {
                topic.setValue(null, this._serverOffset !== null ? this._serverOffset + Date.now() * 1000 : null);
            }
        });
        this._serverOffset = null;
        console.warn(`[NT] Socket closed with code ${event.code}: ${event.reason?.length ? event.reason : `Unknown Reason`}`);

        if (this._started) {
            setTimeout(() => this._connect(), 1000);
        }
    }

    private _onError(): void {
        this._ws?.close();
    }

    private _onMessage(event: MessageEvent): void {
        if (typeof event.data === `string`) {
            this._byteLengthCounter += event.data.length;
            let data: Array<{ method: string; params: any }> = JSON.parse(event.data);
            if (!Array.isArray(data)) {
                console.warn(`[NT] Ignoring Text: top level array not found`);
                return;
            }

            data.forEach((msg, i) => {
                if (typeof msg !== `object`) {
                    console.warn(`[NT] Ignoring Text Parameter: index ${i} of parsed array is not an object`);
                    return;
                }

                if (typeof msg.method !== `string` || typeof msg.params !== `object`) {
                    console.warn(`[NT] Ignoring Text Parameter: index ${i} of parsed array schema mismatch`);
                    return;
                }

                switch (msg.method) {
                    case `announce`:
                        const aName: string = msg.params.name;
                        const aId: number = msg.params.id;
                        const aType: NTTypeString = msg.params.type;

                        const existing = this._topics.get(aName);
                        if (existing) {
                            existing.setId(aId);
                            existing.setType(aType);
                        } else {
                            this._topics.set(aName, new NTTopic(aName, aId, aType));
                        }

                        this._topicMap.update((last) => [...last, [aName, aType]]);
                        break;
                    case `unannounce`:
                        const unName: string = msg.params.name;

                        const topic = this._topics.get(unName);
                        if (topic) {
                            topic.setId(null);
                            topic.setType(null);
                            topic.setValue(null, this._serverOffset !== null ? this._serverOffset + Date.now() * 1000 : null);
                            this._topicMap.update((last) => last.filter((v) => v[0] !== unName));
                        } else {
                            console.warn(`[NT] Ignoring Text Parameter: Unannounced unknown topic ID of ${msg.params.id as number}`);
                        }
                        break;
                    case `properties`:
                        break;
                    default:
                        console.warn(`[NT] Ignoring Text Parameter: index ${i} of parsed array has unknown method`);
                        break;
                }
            });
        } else {
            MessagePack.deserialize(event.data, { multiple: true }).forEach((unpacked: any[]) => {
                const topicId: number = unpacked[0];
                const timestamp: number = unpacked[1];
                const value = unpacked[3];

                if (topicId === -1) {
                    const localTime = Date.now() * 1000;
                    const offset = localTime - value;
                    this._serverOffset = timestamp + offset - localTime;

                    const latency = (offset / 2 / 1000).toFixed(2);
                    this._serverTime = (this._serverOffset + localTime) / 1000000;
                    this._latencyStore.set(latency);
                    console.log(`[NT] Server time is ${this._serverTime.toFixed(3)}s with ${latency}ms latency`);
                } else if (topicId >= 0) {
                    for (const [_, topic] of this._topics) {
                        if (topic.getId() === topicId) {
                            topic.setValue(value, timestamp);
                        }
                    }
                } else {
                    console.warn(`[NT] Ignoring Binary Data: Invalid topic ID of ${topicId}`);
                }
            });

            this._byteLengthCounter += event.data.byteLength;
        }
    }

    private _getSubuid(): number {
        this._subuidNonce++;
        return this._subuidNonce;
    }

    private _getPubuid() {
        return Math.floor(Math.random() * 99999999) + 10000;
    }
}
