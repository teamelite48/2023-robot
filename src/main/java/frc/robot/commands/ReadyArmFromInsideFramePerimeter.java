// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmPreset;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystem.GripperMode;

public class ReadyArmFromInsideFramePerimeter extends SequentialCommandGroup {

  ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
  GripperSubsystem gripperSubsystem = RobotContainer.gripperSubsystem;

  ArmPosition conePosition;
  ArmPosition cubePosition;

  public ReadyArmFromInsideFramePerimeter(ArmPosition conePosition, ArmPosition cubePosition) {
    addCommands(
      new ConditionalCommand(
        whenCommandedToMoveInsideFramePerimeter(),
        whenCommandedToMoveOutsideFramePerimeter(),
        () -> isArmCommandedToMoveInsideFramePerimater()
      )
    );
  }

  private Command whenCommandedToMoveInsideFramePerimeter() {
    return thenMoveArmToPosition();
  }

  private Command whenCommandedToMoveOutsideFramePerimeter() {
    return new ConditionalCommand(
      whenCommandedBellowLowDropZone(),
      whenCommandedAboveHighDropZone(),
      () -> isArmCommandedToMoveBelowLowDropZone()
    );
  }

  private Command whenCommandedBellowLowDropZone() {
    return new SequentialCommandGroup(
      new MoveArmTo(ArmPreset.LOW_DROP_ZONE),
      thenMoveArmToPosition()
    );
  }

  private Command whenCommandedAboveHighDropZone() {
    return new SequentialCommandGroup(
      new MoveArmTo(ArmPreset.HIGH_DROP_ZONE),
      thenMoveArmToPosition()
    );
  }

  private ConditionalCommand thenMoveArmToPosition() {
    return new ConditionalCommand(
      new MoveArmTo(conePosition),
      new MoveArmTo(cubePosition),
      () -> isGripperInConeMode()
    );
  }

  private boolean isArmCommandedToMoveInsideFramePerimater() {

    if (isGripperInConeMode()) {
      return conePosition.x < ArmPreset.FRAME_X_BOUNDARY;
    }
    else {
      return cubePosition.x < ArmPreset.FRAME_X_BOUNDARY;
    }
  }

  private boolean isGripperInConeMode() {
    return gripperSubsystem.getMode() == GripperMode.Cone;
  }

  private boolean isArmCommandedToMoveBelowLowDropZone() {

    if (isGripperInConeMode()) {
      return this.conePosition.y < ArmPreset.LOW_DROP_ZONE.y;
    }
    else {
      return this.cubePosition.y < ArmPreset.LOW_DROP_ZONE.y;
    }
  }
}
