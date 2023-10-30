<script lang="ts">
    import type { Unsubscriber } from "svelte/store";
    import type { NTHierarchy, NTReference, NTType } from "../../lib/NTSvelte";
    import { nt } from "../../ntStores";
    import { createEventDispatcher, onDestroy } from "svelte";

    export let hierarchy: NTHierarchy;
    export let topLevel = true;
    export let parent = ``;

    interface $$Events {
        mouseDownTopic: CustomEvent<NTReference>;
    }

    type Dispatcher<TEvents extends Record<keyof TEvents, CustomEvent<any>>> = {
        [Property in keyof TEvents]: TEvents[Property]["detail"];
    };

    const dispatch = createEventDispatcher<Dispatcher<$$Events>>();
    const handleMouseDown = (topic: any) => {
        dispatch(`mouseDownTopic`, topic);
    };

    const unsubscribers: Unsubscriber[] = [];
    onDestroy(() => {
        for (const unsubscribe of unsubscribers) unsubscribe();
    });

    let detailStorage: Record<string, boolean> = Object.fromEntries(
        Object.entries(localStorage)
            .filter(([k]) => k.startsWith(`nt-list-`))
            .map(([k, v]) => [k, v === `true`]),
    );
    $: Object.keys(detailStorage).forEach((key) => {
        if (!detailStorage[key]) delete localStorage[key];
        else localStorage[key] = `${detailStorage[key]}`;
    });

    let values: Record<string, NTType> = {};
    for (const topic of Object.values(hierarchy)) {
        if (Array.isArray(topic)) {
            const unsubscribe = nt.createSubscription<NTType>(topic[0], null).subscribe((v) => {
                if (v === null) {
                    delete values[topic[0]];
                } else {
                    values = { ...values, ...{ [topic[0]]: v } };
                }
            });
            unsubscribers.push(unsubscribe);
        }
    }

    const format = (value: any) => {
        const string =
            value !== null && value !== undefined ? (typeof value === `object` ? JSON.stringify(value) : value.toString()) : `{Null}`;
        return string.length > 0 ? string : `{Empty String}`;
    };

    const copy = (target: any) => {
        if (!target) return;
        navigator.clipboard.writeText(target.innerText);
    };
</script>

<main>
    <div class="{topLevel ? `top-level` : `norm`}">
        {#each Object.entries(hierarchy).sort((a, b) => a[0].localeCompare(b[0])) as [key, topic]}
            {#if Array.isArray(topic)}
                <div class="data-container">
                    <div class="data-key">
                        <button on:mousedown="{() => handleMouseDown(topic)}"><code>{key}</code></button>
                    </div>
                    <div class="data-type">
                        <code>({topic[1]})</code>
                    </div>
                    <div class="data-value">
                        <button on:click="{(e) => copy(e.target)}"><code>{format(values[topic[0]])}</code></button>
                    </div>
                </div>
            {:else if topLevel}
                <svelte:self hierarchy="{topic}" topLevel="{false}" on:mouseDownTopic />
            {:else}
                <details bind:open="{detailStorage[`nt-list-${parent}/${key}`]}">
                    <summary>{key}</summary>
                    <svelte:self hierarchy="{topic}" topLevel="{false}" parent="{`${parent}/${key}`}" on:mouseDownTopic />
                </details>
            {/if}
        {/each}
    </div>
</main>

<style>
    .top-level,
    .norm {
        text-align: left;
        font-weight: 600;
    }

    .top-level {
        margin: 0.5rem 1rem 1rem 0;
    }

    .norm {
        margin-left: 1rem;
    }

    .data-container {
        display: flex;
        flex-wrap: nowrap;
        gap: 1rem;
    }

    .data-key {
        flex-basis: 80%;
        width: 100%;
        overflow: hidden;
    }

    .data-key > button {
        width: 100%;
        padding: 0.2rem 0.4rem;
        background-color: inherit;
        color: inherit;
        border: 0;
        border-radius: 0.3rem;
        font-size: 0.8rem;
        line-height: 0.8rem;
        text-align: left;
        overflow: hidden;
        text-overflow: ellipsis;
        white-space: nowrap;
        transition: background-color 0.01s ease-out;
    }

    .data-key > button:hover {
        cursor: grab;
        background-color: var(--background-quaternary);
        transition: none;
    }

    .data-key > button:active {
        cursor: grabbing;
    }

    .data-type {
        flex-basis: 30%;
        width: 100%;
        overflow: hidden;
    }

    .data-type > code {
        font-size: 0.8em;
    }

    .data-value {
        flex-grow: 1;
        width: 100%;
        overflow: hidden;
    }

    .data-value > button {
        width: inherit;
        color: inherit;
        padding: 0.15rem 0.4rem;
        text-align: right;
        background-color: var(--background-quaternary);
        border: 0;
        border-radius: 0.3rem;
        font-size: 0.8rem;
        line-height: 0.8rem;
        overflow: hidden;
        text-overflow: ellipsis;
        white-space: nowrap;
    }

    button:hover,
    summary:hover {
        cursor: pointer;
    }
</style>
