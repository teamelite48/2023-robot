// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmPosition;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;

public class MoveArmTo extends CommandBase {

  private final ArmSubsystem armSubsystem = RobotContainer.armSubsystem;
  private final ArmPosition desiredArmPosition;
  private final double threshold = 0.01;

  public MoveArmTo(ArmPosition desiredArmPosition) {
    this.desiredArmPosition = desiredArmPosition;
    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    this.armSubsystem.setArmState(ArmState.Transitioning);
    this.armSubsystem.setPosition(this.desiredArmPosition.x, this.desiredArmPosition.y, this.desiredArmPosition.wristDegrees);
  }

  @Override
  public void execute() {
    if (Robot.isSimulation()) {
      System.out.println(this.armSubsystem.getPosition());
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted == false) {
      this.armSubsystem.setArmState(ArmState.Ready);
    }
  }

  @Override
  public boolean isFinished() {

    Translation2d currentArmPosition = this.armSubsystem.getPosition();

    double xError = Math.abs(this.desiredArmPosition.x - currentArmPosition.getX());
    double yError = Math.abs(this.desiredArmPosition.y - currentArmPosition.getY());

    if (xError < threshold && yError < threshold) {
      return true;
    }
    else {
      return false;
    }
  }
}
