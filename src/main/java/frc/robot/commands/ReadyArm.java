// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmPreset;
import frc.robot.subsystems.arm.ArmSubsystem;


public class ReadyArm extends SequentialCommandGroup {

  ArmSubsystem armSubsystem = RobotContainer.armSubsystem;

  public ReadyArm(ArmPosition conePosition, ArmPosition cubePosition) {

    addCommands(
      new ConditionalCommand(
        new ReadyArmFromInsideFramePerimeter(conePosition, cubePosition),
        new ReadyArmFromOutsideFramePerimeter(conePosition, cubePosition),
        () -> armSubsystem.isArmInsideFramePerimeter()
      )
    );
  }
  
}
