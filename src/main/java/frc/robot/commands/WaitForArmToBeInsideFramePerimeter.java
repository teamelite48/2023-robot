// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem;

public class WaitForArmToBeInsideFramePerimeter extends CommandBase {
  
  ArmSubsystem armsubsystem = RobotContainer.armSubsystem;

  public WaitForArmToBeInsideFramePerimeter() {}

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return armsubsystem.isArmInsideFramePerimeter();
  }
}
