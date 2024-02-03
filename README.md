# GRRBase
A template repository for Java-based robots in the FIRST Robotics Competition. It contains boilerplate for programming with WPILib's command framework, as well as utilities outlined in the packages section below. Additionally, this repository contains GRRDashboard, a Svelte-based dashboard for interacting with and analyzing functions of the robot via Network Tables.

## Code Styling
All Java code follows the styling guide of [Prettier](https://prettier.io/). You can apply these rules via [Spotless](https://github.com/diffplug/spotless/tree/main) using the command `./gradlew spotlessApply`

## Packages

### `org.team340.lib`
Contains an extended subsystem that automatically logs hardware information, as well as bindings to our custom dashboard.

### `org.team340.lib.commands`
Contains a builder class for creating inline commands with a similar API to subclassed commands.

### `org.team340.lib.controller`
Contains a joystick profiler for controllers, as well as a wrapper class for WPILib's `XboxController` class.

### `org.team340.lib.swerve`
Contains a hardware agnostic swerve library with extra features such as a rate limiter that respects the kinematic constraints of the robot.

### `org.team340.lib.util`
Contains various utility classes.

### `org.team340.lib.util.config`
Contains various wrapper classes for saving constants.

### `org.team340.lib.util.config.rev`
Contains configuration builders that safely apply settings to REV hardware.
