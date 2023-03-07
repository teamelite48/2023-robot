// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoTarget extends CommandBase {

  private final DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;
  private final VisionSubsystem visionSubsystem = RobotContainer.visionSubsystem;

  private final double proportionOfError = 1.0 / 3.0;

  public AutoTarget() {
    addRequirements(driveSubsystem, visionSubsystem);
  }

  @Override
  public void initialize() {
    visionSubsystem.enableLed();
  }

  @Override
  public void execute() {
    var x = visionSubsystem.getXError() * proportionOfError;
    driveSubsystem.autoDrive(x, 0, 0);
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
