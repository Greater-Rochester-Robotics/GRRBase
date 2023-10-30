# GRRBase
A template repository for Java-based robots in the FIRST Robotics Competition. It contains boilerplate for programming with WPILib's command framework, as well as utilities outlined in the packages section below. Additionally, this repository contains GRRDashboard, a Svelte-based dashboard for interacting with and analyzing functions of the robot via Network Tables.

## Code Styling
All Java code follows the styling guide of [Prettier](https://prettier.io/). You can apply these rules via [Spotless](https://github.com/diffplug/spotless/tree/main) using the command `./gradlew spotlessApply`

## Packages

#### `org.team340.lib.blacklight`
Contains a simple wrapper for interfacing with a [Blacklight](https://github.com/BR88C/Blacklight).

#### `org.team340.lib.commands`
Contains general purpose utility commands.

#### `org.team340.lib.drivers`
Contains custom drivers for interacting with hardware.

#### `org.team340.lib.subsystem`
Contains `GRRSubsystem`, an extension of WPILib's `Subsystem` that provides additional utilities as well as bindings for GRRDashboard.

#### `org.team340.lib.swerve`
A hardware independent swerve library. It is configured via builders, and includes features such as closed loop per-module heading and velocity control, additional steps to enforce kinematic constraints to reduce scrub, and simulation support.

#### `org.team340.lib.util`
Miscellaneous utilities. Includes an Xbox Controller wrapper, methods for flipping coordinates based on the robot's alliance, math utilities, string utilities, and methods for ensuring safe configuration of Rev hardware.
