import { MessagePack } from "./MessagePack";

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

/**
 * Types supported by Network Tables.
 */
export type NTType = boolean | number | string | Uint8Array | boolean[] | number[] | string[];
/**
 * A string representing a topic's type.
 */
export type NTTypeString = keyof typeof NTTypeCodes;

/**
 * An NT server topic.
 */
interface NTTopic {
    readonly name: string
    readonly id: number
    readonly type: NTTypeString
}

/**
 * An NT subscription that can be subscribed to a single topic.
 */
class NTSubscriber {
    private readonly _topicName: string;
    private readonly _saveHistory: boolean;

    private _history: Map<number, NTType | null> = new Map();
    private _listeners: Set<(value: NTType | null) => void> = new Set();
    private _subuid: number | null = null;
    private _topicId: number | null = null;
    private _value: NTType | null = null;
    private _valueTimestamp: number | null = null;

    /**
     * Create the subscriber.
     * @param topicName The name of the topic.
     * @param saveHistory If the subscriber should save the topic's history.
     */
    constructor (topicName: string, saveHistory: boolean = false) {
        this._topicName = topicName;
        this._saveHistory = saveHistory;
    }

    /**
     * Gets the subscribe JSON and sets the subscriber's internal state to be subscribed.
     * @param subuid The subscription UID to use.
     * @param all If all value changes should be sent to the client.
     * @param periodic The interval the topic should be updated at in seconds.
     */
    public subscribe (subuid: number, all: boolean, periodic: number): Record<string, any> {
        this._subuid = subuid;
        return {
            uid: subuid,
            topics: [this._topicName],
            options: { all, periodic }
        }
    }

    /**
     * Gets the unsubscribe JSON and sets the subscriber's internal state to be unsubscribed.
     * Additionally, this sets the subscriber's value to `null`, updates the listeners, and clears the subscriber's history.
     */
    public unsubscribe (): Record<string, any> {
        const subuid = this._subuid;
        this._subuid = null;
        this._topicId = null;
        if (this._value !== null) this.updateValue(null);
        this._history.clear();
        return { subuid };
    }

    /**
     * This should be called when the subscription's topic is announced.
     * @param topicId The topic ID sent by the server.
     */
    public onAnnounce (topicId: number): void {
        this._topicId = topicId;
    }

    /**
     * This should be called when the subscription's topic is unannounced.
     * Resets the saved topic ID, sets the subscriber's value to `null`, updates the listeners, and clears the subscriber's history.
     */
    public onUnannounce (): void {
        this._topicId = null;
        if (this._value !== null) this.updateValue(null);
        this._history.clear();
    }

    /**
     * Gets the subscriber's topic ID sent by the server.
     * This returns `null` if the topic hasn't received an `announce` message, or if the topic was unannounced.
     */
    public getTopicId (): number | null {
        return this._topicId;
    }

    /**
     * Updates the subscriber's value and updates its listeners with the new value.
     * Also saves the value to the subscriptions's history if enabled.
     * @param value The new value.
     * @param timestamp The value's timestamp in seconds.
     */
    public updateValue (value: NTType | null, timestamp: number | null = null): void {
        if (timestamp === null || timestamp > (this._valueTimestamp ?? 0)) {
            this._value = value;
            this._valueTimestamp = timestamp;
            this._listeners.forEach((listener) => listener(value));
        }

        if (this._saveHistory && timestamp !== null) {
            this._history.set(timestamp, value);
        }
    }

    /**
     * Adds a listener to the subscriber and invokes it with the current value.
     * @param listener The listener to add.
     */
    public addListener (listener: (value: NTType | null) => void): void {
        this._listeners.add(listener);
        listener(this._value);
    }

    /**
     * Removes a listener.
     * @param listener The listener to remove.
     * @returns If the subscriber contains any other listeners.
     */
    public removeListener (listener: (value: NTType | null) => void): boolean {
        this._listeners.delete(listener);
        return this._listeners.size > 0;
    }

    /**
     * Gets the subscriber's history.
     */
    public getHistory (): Map<number, NTType | null> {
        return this._history;
    }

    /**
     * Trims the subscriber's history back to a specified timestamp.
     * @param until Keep history as far back as this timestamp in seconds.
     */
    public trimHistory (until: number) {
        if (!this._saveHistory) return;
        for (const [timestamp] of this._history) {
            if (timestamp < until) this._history.delete(timestamp);
        }
    }
}

/**
 * An NT publisher.
 */
export class NTPublisher {

}

export class NTSvelteClient {
    private static readonly _PORT = 5810;
    private static readonly _ALIVE_CHECK_TIMEOUT = 500;
    private static readonly _RECONNECT_TIMEOUT = 500;
    private static readonly _TIMESTAMP_PERIOD = 250;
    private static readonly _WS_PROTOCOL = `v4.1.networktables.first.wpi.edu`;
    private static readonly _WS_RTT_PROTOCOL = `rtt.networktables.first.wpi.edu`;

    private readonly _appName: string;
    private readonly _defaultPeriodic: number;

    private _connected = false;
    private _started = false;
    private _uri: string;
    private _usage: number = 0;
    private _ws: WebSocket | null = null;
    private _wsRtt: WebSocket | null = null;

    private _subscribers: Map<string, NTSubscriber> = new Map();
    private _serverTopics: Map<string, NTTopic> = new Map();

    constructor(appName: string, defaultPeriodic: number, uri: string) {
        this._appName = appName;
        this._defaultPeriodic = defaultPeriodic;
        this._uri = uri;

        setInterval(() => {}, NTSvelteClient._TIMESTAMP_PERIOD);
    }

    private get _aliveUrl(): string {
        return encodeURI(`http://${this._uri}:${NTSvelteClient._PORT}`);
    }

    private get _wsUrl(): string {
        return encodeURI(`ws://${this._uri}:${NTSvelteClient._PORT}/nt/${this._appName}`)
    }

    /**
     * Connect to the NT server.
     */
    public connect(): void {
        if (!this._started) {
            this._started = true;
            this._connectWs(false);
        }
    }

    /**
     * Sends a message pack packet.
     * @param data The data to send.
     * @param rtt If the packet should be sent to the RTT connection.
     * @returns If the data was sent.
     */
    private _sendMessagePack (data: number[], rtt: boolean): boolean {
        const ws = rtt ? this._wsRtt : this._ws;
        if (ws?.readyState === WebSocket.OPEN) {
            const msg = MessagePack.serialize(data);
            ws.send(msg);
            this._usage += msg.byteLength;
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sends a JSON message.
     * Only used for the primary WS connection, not the RTT connection.
     * @param method The method value to send.
     * @param params JSON parameters.
     * @returns If the data was sent.
     */
    private _sendJSON (method: string, params: any): boolean {
        if (this._ws?.readyState === WebSocket.OPEN) {
            const msg = JSON.stringify([{ method, params }]);
            this._ws.send(msg);
            this._usage += new Blob([msg]).size;
            return true;
        } else {
            return false;
        }
    }

    private async _connectWs(rtt: boolean): Promise<void> {
        if (!rtt) {
            if (!this._started) return;
            let aliveResult: Response | null = null;
    
            try {
                aliveResult = await fetch(this._aliveUrl, { signal: AbortSignal.timeout(NTSvelteClient._ALIVE_CHECK_TIMEOUT) });
            } catch (_) {}
    
            if (!aliveResult?.ok) {
                setTimeout(() => this._connectWs(false), NTSvelteClient._RECONNECT_TIMEOUT);
                return;
            }
        }

        const ws = new WebSocket(this._wsUrl, [!rtt ? NTSvelteClient._WS_PROTOCOL : NTSvelteClient._WS_RTT_PROTOCOL]);
        ws.binaryType = `arraybuffer`;
        this[!rtt ? `_ws` : `_wsRtt`] = ws;

        ws.addEventListener(`open`, () => this._onOpen(rtt));
        ws.addEventListener(`message`, (event: MessageEvent) => this._onMessage(rtt, event));
        if (!rtt) {
            ws.addEventListener(`close`, this._onClose.bind(this));
            ws.addEventListener(`error`, this._onError.bind(this));
        }
    }

    private _onOpen (rtt: boolean): void {
        if (!rtt) {
            this._connected = true;
            this._connectWs(true);
        }

        console.log(`[NTSvelte] ${rtt ? `RTT ` : ``} WebSocket Connected`);
    }
    
    private _onMessage(rtt: boolean, event: MessageEvent): void {

    }

    private _onClose (event: CloseEvent): void {
        this._ws?.close();
        this._ws = null;

        this._wsRtt?.close();
        this._wsRtt = null;
    }

    private _onError (): void {

    }
}
