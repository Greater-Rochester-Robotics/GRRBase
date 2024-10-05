package org.team340.lib.util.rev;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

/**
 * Config builder for {@link SparkLimitSwitch}.
 */
public class SparkLimitSwitchConfig extends RevConfigBase<SparkLimitSwitch> {

    /**
     * Clones this config.
     */
    public SparkLimitSwitchConfig clone() {
        var config = new SparkLimitSwitchConfig();
        config.configSteps.addAll(configSteps);
        return config;
    }

    /**
     * Applies the config to a Spark Max attached limit switch. Note that this is a blocking
     * operation. Errors are printed when calling {@link RevConfigRegistry#burnFlashAll()}.
     * @param sparkMax The Spark Max the limit switch is attached to.
     * @param limitSwitch The limit switch.
     */
    public void apply(CANSparkMax sparkMax, SparkLimitSwitch limitSwitch) {
        applySteps(limitSwitch, "Spark Max (ID " + sparkMax.getDeviceId() + ") Limit Switch");
    }

    /**
     * Applies the config to a Spark Flex limit switch. Note that this is a blocking operation.
     * Errors are printed when calling {@link RevConfigRegistry#burnFlashAll()}.
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
