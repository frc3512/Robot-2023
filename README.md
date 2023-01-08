# FRC Team 3512's 2023 Robot

Source code for the 2022 comp robot: TBD

Source code also for the 2022 practice robot: TBD

## Setup

Install the relevant FRC toolchain for your platform (see
https://github.com/wpilibsuite/allwpilib/releases). Make sure the toolchain
is
placed in `~/wpilib/2022/roborio` (Linux) or
`C:\Users\Public\wpilib\2022\roborio` (Windows).

## Documentation

TBD

## Build options

### Build everything

* `./gradlew build`

This runs a roboRIO and desktop build and runs the desktop tests. This may take a while, so more specific builds (see below) are recommended instead.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at 10.35.12.2, and restarts it.

### Simulation GUI

* `./gradlew simulateJava`

## Logging

Logs are created using WPILib's DataLog framework, with the ability to convert to CSV using the DataLog tool or can be viewed using tools such as AdvantageScope.

## Simulation GUI

The simulation GUI is straightforward but can be read more about [here](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/simulation-gui.html).

## Autonomous mode selection

Open shuffleboard and select the desired autonomous mode from the dropdown menu.
When the indicator next to the menu turns from a red X to a green checkmark, the robot has confirmed the selection.

See [this](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html)
for details on how the robot side works.

## Game

The game for 2022 is called Rapid React, where teams are tasked with shooting cargo into either a lower or upper hub. This year there is the standard 15 second autonomous period. Teams earn points in this period for moving off of the tarmac and earn double the usual points for scoring in the hub. If the alliance can score 5 cargo during autonomous, then the number of cargo needed for the Tele-op ranking point is reduced from 20 to 18. Tele-op continues this gameplay while the robot is controlled through driver input and the cargo points are cut in half. Alliances can earn a ranking point for scoring a certain number of cargo in the hub, be it upper or lower. The number of cargo is dependent on how much cargo was scored during autonomous as stated previously. In Tele-op there is the addition to climb in the hangar. The hangar has four levels of bars the increase in height from the ground and are separated from each other. They cascade over and up towards the driverstation. Robots can traverse up these bars, gaining more points for the higher bar they reach, gaining enough points as an alliance grants a ranking point. Climbing is not prohibited to a certain time during the match, meaning there is no Endgame sequence.

## Unique features

This years robot's unique features include:

- TBD

## Goals of the year
|Status|Goal|Additional Description|
|------|----|----------------------|
|||
|||
|||

## Roster

Students: Adan Silva (Lead), Jonathan Shuman
