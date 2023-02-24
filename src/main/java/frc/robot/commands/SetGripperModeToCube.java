// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem.GripperMode;


public class SetGripperModeToCube extends ParallelCommandGroup {

  private final GripperSubsystem gripperSubsystem = RobotContainer.gripperSubsystem;

  public SetGripperModeToCube() {
    addCommands(
      new InstantCommand(() -> gripperSubsystem.setMode(GripperMode.Cube))
    );
  }
}
