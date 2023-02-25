// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmPreset;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem.GripperMode;


public class ReadyArm extends SequentialCommandGroup {

  private final ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
  private final GripperSubsystem gripperSubsystem = RobotContainer.gripperSubsystem;

  public ReadyArm(ArmPosition conePosition, ArmPosition cubePosition) {

    addCommands(
      new ConditionalCommand(
        new MoveArmTo(ArmPreset.DROP_ZONE),
        new InstantCommand(),
        () -> armSubsystem.getArmState() == ArmState.Stowed
      ),
      new ConditionalCommand(
        new MoveArmTo(conePosition),
        new MoveArmTo(cubePosition),
        () -> gripperSubsystem.getMode() == GripperMode.Cone
      )
    );
  }
}
