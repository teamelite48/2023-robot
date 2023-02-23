// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.GripperMode;


public class SetGripperModeToCone extends ParallelCommandGroup {

  public SetGripperModeToCone() {
    addCommands(
      new InstantCommand(() -> RobotContainer.gripperMode = GripperMode.Cone)
    );
  }
}
