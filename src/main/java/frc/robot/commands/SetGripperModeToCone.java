// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem.GripperMode;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;


public class SetGripperModeToCone extends ParallelCommandGroup {

  private final GripperSubsystem gripperSubsystem = RobotContainer.gripperSubsystem;
  private final VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;
  private final LedSubsystem ledSubsystem = RobotContainer.ledSubsystem;

  public SetGripperModeToCone() {
    addCommands(
      new InstantCommand(() -> visionSubsystem.enableReflectiveTapePipeline()),
      new InstantCommand(() -> gripperSubsystem.setMode(GripperMode.Cone)),
      new InstantCommand(() -> ledSubsystem.setLedToYellow())
    );
  }
}
