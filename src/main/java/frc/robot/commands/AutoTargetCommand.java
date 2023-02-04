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
  public void initialize() {
    visionSubsystem.enableLed();
  }

  @Override
  public void execute() {
    var rotation = visionSubsystem.getXOffset() / 27.0 * 0.333;
    driveSubsystem.autoDrive(0, 0, rotation);
  }


  @Override
  public void end(boolean interrupted) {
    visionSubsystem.disableLed();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
