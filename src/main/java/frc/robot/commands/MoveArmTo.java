// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;

public class MoveArmTo extends CommandBase {

  private final ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
  private final ArmPosition armPosition;
  private final double threshold = 0.001;

  public MoveArmTo(ArmPosition armPosition) {
    this.armPosition = armPosition;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    this.armSubsystem.setArmState(ArmState.Tansitioning);
    this.armSubsystem.setPosition(this.armPosition.x, this.armPosition.y, this.armPosition.wristDegrees);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    this.armSubsystem.setArmState(armPosition.armState);
  }

  @Override
  public boolean isFinished() {

    Translation2d armPosition = this.armSubsystem.getArmPosition();

    double xError = Math.abs(this.armPosition.x - armPosition.getX());
    double yError = Math.abs(this.armPosition.y - armPosition.getY());

    if (xError < threshold && yError < threshold) {
      return true;
    }
    else {
      return false;
    }
  }
}
