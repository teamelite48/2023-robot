// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmPreset;
import frc.robot.subsystems.arm.ArmSubsystem;

public class StowArm extends SequentialCommandGroup {

  private final ArmSubsystem armSubsystem = RobotContainer.armSubsystem;

  public StowArm() {
    addCommands(
      new SequentialCommandGroup(
          new ConditionalCommand(
            new MoveArmTo(ArmPreset.LOW_DROP_ZONE),
            new MoveArmTo(ArmPreset.HIGH_DROP_ZONE),
            () -> armSubsystem.getPosition().getY() <  ArmPreset.SCORE_CUBE_MID.y
          ),
          new MoveArmTo(ArmPreset.STOWED_CONE)
        )
    );
  }
}
