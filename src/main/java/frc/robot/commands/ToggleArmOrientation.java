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

public class ToggleArmOrientation extends SequentialCommandGroup {

  ArmSubsystem armSubsystem = RobotContainer.armSubsystem;

  public ToggleArmOrientation() {
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new InstantCommand(() -> armSubsystem.toggleOrientation()),
          new ReadyArm(ArmPreset.STOWED_CONE, ArmPreset.STOWED_CONE)
        ),
        new DoNothing(),
        () -> armSubsystem.getArmState() == ArmState.Ready && armSubsystem.isArmInsideFramePerimeter()
      )
    );
  }
}
