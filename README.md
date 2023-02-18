# FRC Team 3512's 2023 Robot

Source code for the 2023 comp robot: Thanatos

Source code also for the 2023 practice robot: Atlas

## Setup

Install the relevant FRC toolchain for your platform (see
https://github.com/wpilibsuite/allwpilib/releases). Make sure the toolchain
is
placed in `~/wpilib/2022/roborio` (Linux) or
`C:\Users\Public\wpilib\2022\roborio` (Windows).

## Build options

### Build everything

* `./gradlew build`

This builds code for the roboRIO and tests (if present), but doesn't deploy it to the robot.

### Deploy

* `./gradlew deploy`

This runs a roboRIO build if needed, copies the resulting binary to a roboRIO at 10.35.12.2, and restarts it.

### Simulation GUI

* `./gradlew simulateJava`
Runs the simulation GUI for testing robot code without the real hardware.

### Format code
* `./gradlew spotlessApply`
Beautifies your code to make it easier to read. Required for builds/GitHub CLI to pass.

## Telemetry

Telemetry values are sent using NT4, with the ability for them to be disabled during competitions to ensure the maximum amount of network bandwitdth. When tuning, they can be viewed in the standard dashboard program like Shuffleboard or opened up in specific viewing programs like AdvantageScope.

## Simulation GUI

The simulation GUI is straightforward but can be read more about [here](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-simulation/simulation-gui.html).

## Autonomous mode selection

Open shuffleboard and select the desired autonomous mode from the dropdown menu.
When the indicator next to the menu turns from a red X to a green checkmark, the robot has confirmed the selection.

See [this](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html)
for details on how the robot side works.

## Game

The game for 2023 is called Charged Up, where teams are tasked with placing inflated cubes and cones into their own respective scoring nodes constructed together as a game element called the Grid. This year there is the standard 15 second autonomous period. Teams earn points in this period for leaving the Community Zone and earn more than the usual points for scoring in game pieces on these nodes. With one robot of an alliance, it is able to engage or dock with the Charge Station to earn additional points. Tele-op continues with teams resuming to scoring game pieces into the nodes, with the robot is controlled through driver input and with the point values reduced. If 3 objects are placed on in an adjustant row, then a Link is created and scores additional points. If an alliance creates 5+ links during a match, a ranking point is awarded. If a link is made in the middle Grid, also known as the Coopertition Grid, then it can allow for a ranking point while also lowering the threshold for the link ranking point from 5 to 4 for both alliances. While a proper endgame procedure doesn't exist for this year, teams are able to drive onto the Charge Station and park on it to score points while being able to score additional points by parking and leveling the Charge Station as well towards the end of a match.

## Unique features

This years robot's unique features include:

- Swerve drive with SDS MK4 L2 modules using CANCoders and NEOs
- New vision camera system for use with AprilTags
- Elevator using NEOs and a REV Through Bore encoder set to absolute mode
- Arm using NEOs and a REV Through Bore encoder set to absolute mode
- Intake similar to the 2023 Everybot to pick up cones/cubes

## Goals of the year
|Status|Goal|Additional Description|
|------|----|----------------------|
|Yes|Swerve Drive|Operational auton/teleop swerve code
|Yes|AprilTags|Read and process AprilTags using PhotonVision
|Yes|Pose Estimation|Add in pose estimation using AprilTags.
|Yes|Pose based Alignment|Use pose estimation to drive onto a certain point on the field.
|No|Manual and Automated Elevator|Elevator subsystem with both manual and automated control.
|No|Manual and Automated Arm|Arm subsystem with both manual and automated control.
|No|AutoDrop|Sequence for automatically scoring onto the correct node.

## Roster

Students: Adan Silva (Lead), Jonathan Shuman
