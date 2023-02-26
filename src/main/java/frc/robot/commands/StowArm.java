// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmPreset;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;

public class StowArm extends SequentialCommandGroup {

  private final ArmSubsystem armSubsystem = RobotContainer.armSubsystem;

  public StowArm() {
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new MoveArmTo(ArmPreset.DROP_ZONE),
          new MoveArmTo(ArmPreset.STOWED)
        ),
        new InstantCommand(),
        () -> armSubsystem.getArmState() == ArmState.Ready
      )
    );
  }
}