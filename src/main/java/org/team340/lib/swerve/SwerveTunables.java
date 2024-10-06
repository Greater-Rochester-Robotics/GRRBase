package org.team340.lib.swerve;

import org.team340.lib.dashboard.Tunable;
import org.team340.lib.swerve.config.SwerveConfig;

/**
 * Utility class for constructing tunables for the {@link SwerveAPI}.
 */
final class SwerveTunables {

    private SwerveTunables() {
        throw new AssertionError("This is a utility class!");
    }

    /**
     * Enables publishing tunables for adjustment of the API's constants.
     * @param name The parent name for the tunables in NetworkTables.
     * @param api The swerve API.
     */
    public static void initialize(String name, SwerveAPI api) {
        SwerveConfig config = api.config;

        Tunable.doubleValue(name + "/velocity", config.velocity, v -> config.velocity = v);
        Tunable.doubleValue(name + "/slipAccel", config.slipAccel, v -> config.slipAccel = v);
        Tunable.doubleValue(name + "/torqueAccel", config.torqueAccel, v -> config.torqueAccel = v);
        Tunable.doubleValue(name + "/angularAccel", config.angularAccel, v -> config.angularAccel = v);
        Tunable.doubleValue(name + "/driverVel", config.driverVel, v -> config.driverVel = v);
        Tunable.doubleValue(name + "/driverVelExp", config.driverVelExp, v -> config.driverVelExp = v);
        Tunable.doubleValue(name + "/driverAngularVel", config.driverAngularVel, v -> config.driverAngularVel = v);
        Tunable.doubleValue(name + "/driverAngularVelExp", config.driverAngularVelExp, v ->
            config.driverAngularVelExp = v
        );
        Tunable.doubleValue(name + "/discretizationPeriod", config.discretizationPeriod, v ->
            config.discretizationPeriod = v
        );

        Tunable.doubleValue(name + "/moveMotors/kP", config.movePID[0], v -> {
            System.out.println(v);
            config.movePID[0] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/moveMotors/kI", config.movePID[1], v -> {
            config.movePID[1] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/moveMotors/kD", config.movePID[2], v -> {
            config.movePID[2] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/moveMotors/iZone", config.movePID[3], v -> {
            config.movePID[3] = v;
            reapplyGains(true, api);
        });

        Tunable.doubleValue(name + "/moveMotors/kS", config.moveFF[0], v -> {
            config.moveFF[0] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/moveMotors/kV", config.moveFF[1], v -> {
            config.moveFF[1] = v;
            reapplyGains(true, api);
        });

        Tunable.doubleValue(name + "/turnMotors/kP", config.turnPID[0], v -> {
            config.turnPID[0] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/turnMotors/kI", config.turnPID[1], v -> {
            config.turnPID[1] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/turnMotors/kD", config.turnPID[2], v -> {
            config.turnPID[2] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/turnMotors/iZone", config.turnPID[3], v -> {
            config.turnPID[3] = v;
            reapplyGains(false, api);
        });
    }

    /**
     * Re-applies PID and FF gains to motors from the swerve config.
     * Used for setting new gains after the config has been mutated.
     * @param moveMotors {@code true} reapplies to all move motors, {@code false} reapplies to all turn motors.
     * @param api The swerve API.
     */
    private static void reapplyGains(boolean moveMotors, SwerveAPI api) {
        for (var module : api.modules) {
            if (moveMotors) {
                module.moveMotor.reapplyGains();
            } else {
                module.turnMotor.reapplyGains();
            }
        }
    }
}
