<script lang="ts">
    import { derived } from "svelte/store";
    import field24 from "../assets/field24.png";
    import { FIELD_HEIGHT, FIELD_WIDTH } from "../constants";
    import { AutosActive, AutosOptions, AutosSelected, RobotBlueAlliance } from "../ntStores";

    // import field22 from '../assets/field22.png';
    // import field23 from '../assets/field23.png';
    const field = field24;

    // Option typings.
    type Option = {
        id: string;
        label: string;
        points: Array<[number, number, number]>;
    };

    // Parses options from NT value.
    const options = derived(AutosOptions, ($value) => {
        return (
            $value?.map((option) => {
                try {
                    const parsed = { ...JSON.parse(option) };
                    if (![`id`, `label`, `points`].every((v) => Object.keys(parsed).includes(v)))
                        throw new Error(`Invalid auto option: ${option}`);
                    return parsed;
                } catch (error) {
                    return null;
                }
            }) ?? []
        ).filter((v) => v !== null) as Option[];
    });
</script>

<main>
    <div class="autos-container">
        {#if $options.length}
            {#each $options as { id, label, points }}
                <!-- An auto selection. -->
                <button
                    class="auto-selection"
                    class:auto-selection-client="{id === $AutosSelected}"
                    class:auto-selection-robot="{id === $AutosActive}"
                    on:keydown="{() => {}}"
                    on:click="{() => {
                        $AutosSelected = id;
                    }}"
                >
                    <!-- The selection's path. -->
                    <div class="auto-selection-path">
                        <!-- The field. -->
                        <img src="{field}" alt="field" />

                        <!-- The path. -->
                        <svg
                            viewBox="{0} {0} {FIELD_WIDTH} {FIELD_HEIGHT}"
                            fill="none"
                            stroke="var(--auto-line)"
                            stroke-width="0.06"
                            stroke-linecap="round"
                            stroke-linejoin="round"
                        >
                            {#each points as [x, y, _], i}
                                {#if i < points.length - 1}
                                    <line
                                        x1="{$RobotBlueAlliance ? x : FIELD_WIDTH - x}"
                                        y1="{FIELD_HEIGHT - y}"
                                        x2="{$RobotBlueAlliance ? points[i + 1][0] : FIELD_WIDTH - points[i + 1][0]}"
                                        y2="{FIELD_HEIGHT - points[i + 1][1]}"
                                    ></line>
                                {/if}
                            {/each}
                        </svg>
                    </div>

                    <!-- The selection's label. -->
                    <p class="auto-selection-label">{label}</p>
                </button>
            {/each}
        {:else}
            <em style="margin-top: 20%; font-size: 3rem;">No available auto options</em>
        {/if}
    </div>
</main>

<style>
    @keyframes selecting {
        0% {
            border-color: var(--accent-color-transparent);
        }
        50% {
            border-color: var(--accent-color);
        }
        100% {
            border-color: var(--accent-color-transparent);
        }
    }

    .autos-container {
        display: flex;
        margin: 2rem;
        justify-content: center;
        flex-wrap: wrap;
        gap: 1.5rem;
    }

    .auto-selection {
        background-color: var(--background-tertiary);
        padding: 2rem 2rem 0 2rem;
        border: 3px solid transparent;
        border-radius: 1.5rem;
        cursor: pointer;
        color: inherit;
        font-size: inherit;
        font-family: inherit;
        box-shadow: 0.2rem 0.2rem 0.5rem var(--background-shadow);
        transition:
            border-color 0.1s,
            transform 0.2s,
            box-shadow 0.2s;
    }

    .auto-selection-client {
        will-change: filter;
        animation-name: selecting;
        animation-duration: 1.5s;
        animation-iteration-count: infinite;
    }

    .auto-selection-robot {
        animation-name: none !important;
        border-color: var(--accent-color);
    }

    .auto-selection:hover,
    .auto-selection-robot {
        box-shadow: 0.3rem 0.3rem 0.5rem var(--background-shadow);
        transform: scale(1.02);
    }

    .auto-selection-label {
        font-weight: 600;
    }

    .auto-selection-path {
        position: relative;
        display: inline-block;
    }

    .auto-selection-path img {
        display: block;
        width: 24rem;
    }

    .auto-selection-path svg {
        position: absolute;
        top: 0;
        left: 0;
    }
</style>
