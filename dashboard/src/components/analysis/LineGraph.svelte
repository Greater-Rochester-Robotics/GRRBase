<script lang="ts">
    import type { Unsubscriber } from "svelte/store";
    import Chart from "chart.js/auto";
    import zoomPlugin, { zoom } from "chartjs-plugin-zoom";
    import { nt } from "../../ntStores";
    import { onDestroy, onMount } from "svelte";
    import type { NTReference } from "../../lib/NTSvelte";

    export let index: number;
    export let references: NTReference[];

    const colors = [`red`, `orange`, `yellow`, `green`, `teal`, `blue`, `indigo`, `purple`, `pink`, `brown`, `white`] as const;
    let colorsNonce = 0;

    let internalReferences: Array<{
        key: string;
        unsubscribe: Unsubscriber;
        left: boolean;
        color: (typeof colors)[number];
        data: Array<{ x: number; y: number }>;
    }> = [];

    $: {
        for (const reference of references) {
            if (internalReferences.find((r) => r.key === reference[0])) continue;
            else {
                let newReference: (typeof internalReferences)[number] | null = null;
                const pollHistory = (): Array<{ x: number; y: number }> => {
                    return Array.from(nt.getTopicHistoryMap(reference[0]).entries())
                        .filter(([_, value]) => value !== null)
                        .map(([timestamp, value]) => ({ x: timestamp / 1e6, y: value as number }));
                };

                newReference = {
                    key: reference[0],
                    unsubscribe: nt.createSubscription<number>(reference[0], null).subscribe(() => {
                        if (newReference) newReference.data = pollHistory();
                    }),
                    left: true,
                    color: colors[colorsNonce % colors.length],
                    data: pollHistory(),
                };

                internalReferences = [...internalReferences, newReference];
                colorsNonce++;
            }
        }

        if (references.length < internalReferences.length) {
            internalReferences = [...internalReferences.filter(({ key }) => references.find(([k]) => k === key))];
        }
    }

    let ctx: HTMLCanvasElement;
    let ctxParent: HTMLElement;
    let ctxStyle: CSSStyleDeclaration;
    let interval: number | null = null;
    onMount(() => {
        ctxParent = ctx.parentElement!;
        ctxStyle = getComputedStyle(ctx);

        Chart.register(zoomPlugin);
        const chart = new Chart(ctx, {
            type: `line`,
            data: { datasets: [] },
            options: {
                animation: false,
                parsing: false,
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    mode: "nearest",
                    axis: `x`,
                    intersect: false,
                },
                scales: {
                    x: {
                        type: `linear`,
                        grid: { color: ctxStyle.getPropertyValue(`--graph-grid`) },
                        ticks: {
                            maxRotation: 0,
                            autoSkip: true,
                            color: ctxStyle.getPropertyValue(`--text-secondary`),
                        },
                    },
                    y0: {
                        position: `left`,
                        grid: { color: ctxStyle.getPropertyValue(`--graph-grid`) },
                        ticks: { color: ctxStyle.getPropertyValue(`--text-secondary`) },
                    },
                    y1: {
                        position: `right`,
                        display: false,
                        grid: { display: false },
                        ticks: { color: ctxStyle.getPropertyValue(`--text-secondary`) },
                    },
                },
                plugins: {
                    tooltip: {
                        callbacks: {
                            title: (item) => `${(((item[0]?.raw as any)?.x ?? 0) as number).toFixed(4)}s`,
                            label: (item) => `${item.dataset.label?.split(`/`).at(-1)}: ${(item.raw as any)?.y}`,
                        },
                    },
                    legend: {
                        display: false,
                        onClick: (_, item) => {
                            const dataset = chart.data.datasets[item.datasetIndex ?? -1];
                            const internalReference = internalReferences.find(({ key }) => key === dataset.label);
                            if (internalReference) {
                                internalReference.color = `orange`;
                            }
                        },
                    },
                    zoom: {
                        pan: { enabled: true },
                        zoom: {
                            scaleMode: `xy`,
                            wheel: { enabled: true },
                        },
                    },
                },
            },
        });

        interval = setInterval(() => {
            chart.data.datasets = internalReferences.map((r) => {
                const color = ctxStyle.getPropertyValue(`--graph-${r.color}`);

                return {
                    label: r.key,
                    data: r.data,
                    radius: 0,
                    tension: 0,
                    borderWidth: 1.5,
                    borderColor: color,
                    backgroundColor: color,
                    xAxisID: `x`,
                    yAxisID: r.left ? `y0` : `y1`,
                };
            });

            chart.update();
        }, 100);
    });

    onDestroy(() => {
        if (interval !== null) clearInterval(interval);
        internalReferences.forEach((r) => r.unsubscribe());
    });
</script>

<main>
    <div class="canvas-container">
        <canvas data-index="{index}" class="line-chart" bind:this="{ctx}"></canvas>
    </div>
</main>

<style>
    .canvas-container {
        height: 0;
        width: 0;
        min-height: 100%;
        min-width: 100%;
    }
</style>
