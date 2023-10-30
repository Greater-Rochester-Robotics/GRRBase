<script lang="ts">
    import LineGraph from "../components/analysis/LineGraph.svelte";
    import NetworkTablesList from "../components/analysis/NetworkTablesList.svelte";

    import type { NTReference } from "../lib/NTSvelte";
    import { NTTopicMapHierarchy } from "../ntStores";

    let grabbed: NTReference | null = null;
    let visualizations: Array<[NTReference[], LineGraph]> = [];

    function handleMouseMove(event: MouseEvent) {
        if (grabbed !== null) {
            const grabbedElement = document.getElementById("grabbed-element");
            if (!grabbedElement) return;

            if (grabbedElement.style.display === `none`) grabbedElement.style.display = `block`;

            grabbedElement.style.top = `${event.clientY - (grabbedElement?.clientHeight ?? 0) / 2}px`;
            grabbedElement.style.left = `${event.clientX - (grabbedElement?.clientWidth ?? 0) / 2}px`;
        }
    }

    function handleMouseUp(event: MouseEvent) {
        if (event.target && grabbed) {
            if ((event.target as HTMLElement).id === `network-tables-visualization-container`) {
                const target = document.getElementById(`network-tables-visualization-container`);
                if (!target) return;

                if (grabbed[1] === `double` || grabbed[1] === `float` || grabbed[1] === `int`) {
                    const component = new LineGraph({
                        target,
                        props: {
                            index: visualizations.length,
                            references: [grabbed],
                        },
                    });

                    visualizations.push([[grabbed], component]);
                } else if (grabbed[1] === `double[]` || grabbed[1] === `float[]` || grabbed[1] === `int[]`) {
                } else if (grabbed[1] === `boolean` || grabbed[1] === `boolean[]`) {
                } else if (grabbed[1] === `string` || grabbed[1] === `string[]`) {
                }
            } else if ((event.target as HTMLElement).tagName === `CANVAS`) {
                const i = Number((event.target as HTMLElement).getAttribute(`data-index`) ?? `-1`);
                if (visualizations[i]) {
                    const newRef = [...visualizations[i][0], grabbed];
                    visualizations[i][1].$set({ references: newRef });
                    visualizations[i][0] = newRef;
                }
            }
        }

        const grabbedElement = document.getElementById("grabbed-element");
        if (grabbedElement) grabbedElement.style.display = `none`;
        grabbed = null;
    }
</script>

<main>
    <!-- svelte-ignore a11y-no-static-element-interactions -->
    <div class="network-tables-container" on:mousemove="{handleMouseMove}" on:mouseup="{handleMouseUp}">
        <div class="network-tables-list-container">
            <NetworkTablesList hierarchy="{$NTTopicMapHierarchy}" on:mouseDownTopic="{({ detail }) => (grabbed = detail)}" />
        </div>
        <div id="network-tables-visualization-container" class="network-tables-visualization-container"></div>
    </div>

    <div id="grabbed-element" class="grabbed-element" style="display: none">
        <code>{grabbed?.[0] ?? `N/A`} ({grabbed?.[1] ?? `null`})</code>
    </div>
</main>

<style>
    main {
        height: 100%;
        /* overflow: hidden; */
    }

    .network-tables-container {
        display: flex;
        height: calc(100% - 2rem);
        padding: 1rem;
        gap: 1rem;
        flex-wrap: wrap;
    }

    .network-tables-container > * {
        background-color: var(--background-tertiary);
        border-radius: 1rem;
    }

    .network-tables-list-container {
        width: 25%;
        height: 100%;
        overflow-x: hidden;
        overflow-y: scroll;
    }

    .network-tables-visualization-container {
        display: flex;
        padding: 1rem;
        justify-content: center;
        align-content: stretch;
        align-items: stretch;
        flex-direction: column;
        gap: 1rem;
        flex-grow: 1;
    }

    :global(.network-tables-visualization-container > *) {
        padding: 1rem;
        border-radius: 1rem;
        background-color: var(--background-quaternary);
        flex-shrink: 1;
        flex-grow: 1;
    }

    .grabbed-element {
        position: fixed;
        padding: 4px 8px;
        border-radius: 4px;
        background-color: var(--background-quaternary);
        color: var(--text-primary);
        line-height: 1rem;
        pointer-events: none;
        z-index: 1000 !important;
    }

    .grabbed-element > code {
        text-overflow: clip;
        white-space: nowrap;
    }
</style>
