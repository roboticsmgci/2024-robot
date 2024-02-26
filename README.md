# FRC 8574 (Audeamus)'s 2024 Testbot

## Installation

1. Follow [this guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) to install the latest version of WPILib.

2. Make sure you have Git installed. If you're on Windows, you can install Git using [this installer](https://git-scm.com/download/win).

3. Clone this repository.

4. Open the `2024-testbot-new` folder in `2024 WPILib VS Code`.

5. Press `Ctrl + Shift + P` to open the command palette

6. **To run the simulation**: search and select `WPILib: Simulate Robot Code` and wait for the process to complete. **To deploy the code to the robot**: connect to the robot, then search and select `WPILib: Deploy Robot Code` and wait for the process to complete.

## Additional Setup

- To edit the paths and autos, install [FRC PathPlanner](https://www.microsoft.com/store/productId/9NQBKB5DW909?ocid=pdpshare).
- To use the Driver Station and to image the roboRIO, install the FRC Game Tools using [these instructions](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html).

## `Pose2d`, `Rotation2d`, and `Translation2d` Values

- Readings directly from the gyro sensor (e.g., using `m_gyro.getAngle()` or `m_gyro.getRotation2d()`) are increasing going CW. Readings using `getRobotHeadingDegrees` in `DriveSubsystem` and values in `Pose2d` and `Rotation2d` are increasing going CCW.
- For `Translation2d` and `Pose2d` values relative to the robot, positive x values represent moving toward the front of the robot whereas positive y values represent moving toward the left of the robot.
- For `Pose2d` values on the field, the RED SOURCE is at the (0, 0) corner and the RED AMP is closest to the (16.54, 8.21) corner.