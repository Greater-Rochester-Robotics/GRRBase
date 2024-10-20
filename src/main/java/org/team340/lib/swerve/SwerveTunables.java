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

        Tunable.doubleValue(name + "/Limits/velocity", config.velocity, v -> config.velocity = v);
        Tunable.doubleValue(name + "/Limits/slipAccel", config.slipAccel, v -> config.slipAccel = v);
        Tunable.doubleValue(name + "/Limits/torqueAccel", config.torqueAccel, v -> config.torqueAccel = v);
        Tunable.doubleValue(name + "/Limits/angularAccel", config.angularAccel, v -> config.angularAccel = v);

        Tunable.doubleValue(name + "/DriverProfile/velocity", config.driverVel, v -> config.driverVel = v);
        Tunable.doubleValue(name + "/DriverProfile/velocityExp", config.driverVelExp, v -> config.driverVelExp = v);
        Tunable.doubleValue(name + "/DriverProfile/angularVel", config.driverAngularVel, v ->
            config.driverAngularVel = v
        );
        Tunable.doubleValue(name + "/DriverProfile/angularVelExp", config.driverAngularVelExp, v ->
            config.driverAngularVelExp = v
        );

        Tunable.doubleValue(name + "/discretizationPeriod", config.discretizationPeriod, v ->
            config.discretizationPeriod = v
        );

        Tunable.doubleValue(name + "/MoveMotors/kP", config.movePID[0], v -> {
            System.out.println(v);
            config.movePID[0] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/MoveMotors/kI", config.movePID[1], v -> {
            config.movePID[1] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/MoveMotors/kD", config.movePID[2], v -> {
            config.movePID[2] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/MoveMotors/iZone", config.movePID[3], v -> {
            config.movePID[3] = v;
            reapplyGains(true, api);
        });

        Tunable.doubleValue(name + "/MoveMotors/kS", config.moveFF[0], v -> {
            config.moveFF[0] = v;
            reapplyGains(true, api);
        });
        Tunable.doubleValue(name + "/MoveMotors/kV", config.moveFF[1], v -> {
            config.moveFF[1] = v;
            reapplyGains(true, api);
        });

        Tunable.doubleValue(name + "/TurnMotors/kP", config.turnPID[0], v -> {
            config.turnPID[0] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/TurnMotors/kI", config.turnPID[1], v -> {
            config.turnPID[1] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/TurnMotors/kD", config.turnPID[2], v -> {
            config.turnPID[2] = v;
            reapplyGains(false, api);
        });
        Tunable.doubleValue(name + "/TurnMotors/iZone", config.turnPID[3], v -> {
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
