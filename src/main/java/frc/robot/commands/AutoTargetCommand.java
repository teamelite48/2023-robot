// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoTargetCommand extends CommandBase {

  private final DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;
  private final VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

  public AutoTargetCommand() {
    addRequirements(driveSubsystem, visionSubsystem);

  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driveSubsystem.drive(0, 0, visionSubsystem.getXOffset() / 32.0 * 0.33, false);
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
