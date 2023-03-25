// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalance extends CommandBase {

  DriveSubsystem driveSubsystem;

  public AutoBalance() {
    driveSubsystem = RobotContainer.driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    var pitch = driveSubsystem.getPitch();
    var speed = 0.00984375 * pitch;
    driveSubsystem.autoDrive(0, speed, 0);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDrive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}