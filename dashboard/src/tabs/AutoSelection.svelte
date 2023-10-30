<script lang="ts">
    import field23 from "../assets/field23.png";
    import { FIELD_HEIGHT, FIELD_WIDTH } from "../constants";
    import { AutosActive, AutosOptions, AutosSelected, RobotBlueAlliance } from "../ntStores";

    // import field22 from '../assets/field22.png';
    const field = field23;

    // Option typings.
    type ControlPoint = { x: number; y: number };
    type Option = {
        label: string;
        splines: Array<{
            p0: ControlPoint;
            p1: ControlPoint | null;
            p2: ControlPoint | null;
            p3: ControlPoint | null;
        }>;
        raw: string;
    };

    // Load options.
    let options: Option[] = [];
    $: options =
        $AutosOptions?.map((option) => {
            try {
                return {
                    ...JSON.parse(option),
                    raw: option,
                };
            } catch (error) {
                console.error(error);
            }
        }) ?? [];

    // Helper for mirroring the field based on the robot's alliance.
    $: am = $RobotBlueAlliance ? 1 : -1;
</script>

<main>
    <div class="autos-container">
        {#if options.length}
            {#each options as { label, splines, raw }}
                <!-- An auto selection. -->
                <button
                    class="auto-selection"
                    class:auto-selection-client="{raw === $AutosSelected}"
                    class:auto-selection-robot="{raw === $AutosActive}"
                    on:keydown="{() => {}}"
                    on:click="{() => {
                        $AutosSelected = raw;
                    }}"
                >
                    <!-- The selection's path. -->
                    <div class="auto-selection-path">
                        <!-- The field. -->
                        <img src="{field}" alt="field" />

                        <!-- The path. -->
                        <svg
                            viewBox="{$RobotBlueAlliance ? 0 : -FIELD_WIDTH} {-FIELD_HEIGHT} {FIELD_WIDTH} {FIELD_HEIGHT}"
                            fill="none"
                            stroke="var(--auto-spline)"
                            stroke-width="0.06"
                            stroke-linecap="round"
                            stroke-linejoin="round"
                        >
                            {#each splines as spline}
                                {#if spline.p1 && spline.p2 && spline.p3}
                                    <path
                                        d="M {am * spline.p0.x} {-spline.p0.y} C {am * spline.p1.x} {-spline.p1.y}, {am *
                                            spline.p2.x} {-spline.p2.y}, {am * spline.p3.x} {-spline.p3.y}"
                                    ></path>
                                {/if}

                                <circle
                                    cx="{am * spline.p0.x}"
                                    cy="{-spline.p0.y}"
                                    r="0.15"
                                    fill="var(--auto-cp-{$RobotBlueAlliance ? `blue` : `red`})"
                                ></circle>
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
