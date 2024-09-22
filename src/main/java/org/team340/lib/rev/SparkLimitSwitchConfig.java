package org.team340.lib.rev;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

/**
 * Config builder for {@link SparkLimitSwitch}.
 */
public class SparkLimitSwitchConfig extends RevConfigBase<SparkLimitSwitch> {

    /**
     * Creates an empty config.
     */
    public SparkLimitSwitchConfig() {}

    /**
     * Creates a config that copies the config steps from the base provided.
     * @param base The config to copy the steps from.
     */
    private SparkLimitSwitchConfig(RevConfigBase<SparkLimitSwitch> base) {
        super(base);
    }

    /**
     * Clones this config.
     */
    public SparkLimitSwitchConfig clone() {
        return new SparkLimitSwitchConfig(this);
    }

    /**
     * Applies the config to a Spark Max attached limit switch.
     * @param sparkMax The Spark Max the limit switch is attached to.
     * @param limitSwitch The limit switch.
     */
    public void apply(CANSparkMax sparkMax, SparkLimitSwitch limitSwitch) {
        applySteps(limitSwitch, "Spark Max (ID " + sparkMax.getDeviceId() + ") Limit Switch");
    }

    /**
     * Applies the config to a Spark Flex limit switch.
     * @param sparkFlex The Spark Flex the limit switch is attached to.
     * @param limitSwitch The limit switch.
     */
    public void apply(CANSparkFlex sparkFlex, SparkLimitSwitch limitSwitch) {
        applySteps(limitSwitch, "Spark Flex (ID " + sparkFlex.getDeviceId() + ") Limit Switch");
    }

    /**
     * Enables or disables controller shutdown based on the limit switch.
     * @param enable Enable/disable motor shutdown based on the limit switch state. This does not affect the result of the get() command.
     */
    public SparkLimitSwitchConfig enableLimitSwitch(boolean enable) {
        addStep(
            limitSwitch -> limitSwitch.enableLimitSwitch(enable),
            limitSwitch -> limitSwitch.isLimitSwitchEnabled() == enable,
            "Enabled"
        );
        return this;
    }
}
