<script lang="ts">
    import field23 from "../assets/field23.png";
    import { FIELD_HEIGHT, FIELD_WIDTH } from "../../constants";
    import { FieldModules, FieldRobot, RobotBlueAlliance } from "../../ntStores";

    // import field22 from '../assets/field22.png';
    const field = field23;
</script>

<main>
    <!-- The selection's path. -->
    <div class="field">
        <!-- The field. -->
        <img src="{field}" alt="field" />

        <!-- The path. -->
        <svg
            viewBox="0 {-FIELD_HEIGHT} {FIELD_WIDTH} {FIELD_HEIGHT}"
            fill="none"
            stroke="var(--auto-spline)"
            stroke-width="0.06"
            stroke-linecap="round"
            stroke-linejoin="round"
        >
            {#each $FieldModules as module}
                <circle
                    cx="{module?.[0] ?? 0}"
                    cy="{-(module?.[1] ?? 0)}"
                    r="0.1"
                    fill="var(--auto-cp-{$RobotBlueAlliance ? `blue` : `red`})"
                ></circle>
            {/each}

            <circle
                cx="{$FieldRobot?.[0] ?? -1}"
                cy="{-($FieldRobot?.[1] ?? -1)}"
                r="0.15"
                fill="var(--auto-cp-{$RobotBlueAlliance ? `blue` : `red`})"
            ></circle>
        </svg>
    </div>
</main>

<style>
    main {
        width: inherit;
    }

    .field {
        position: relative;
        display: inline-block;
        width: inherit;
    }

    .field img {
        display: block;
        width: inherit;
    }

    .field svg {
        position: absolute;
        top: 0;
        left: 0;
    }
</style>
