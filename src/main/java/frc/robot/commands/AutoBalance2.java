// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalance2 extends CommandBase {

  DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

  double metersToCenter = 0.535;
  double onRampPitch = 14.0;
  double initialOnRampXPos = 0.0;
  boolean isOnRamp = false;
  double speed = 0.25;

  public AutoBalance2() {
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    isOnRamp = false;
    initialOnRampXPos = 0.0;
  }

  @Override
  public void execute() {

    driveSubsystem.autoDrive(0, -speed, 0);

    var currentPitch = driveSubsystem.getPitch();

    if (isOnRamp == false && 13.5 < currentPitch && currentPitch < 14) {
      initialOnRampXPos = driveSubsystem.getOdometry().getPoseMeters().getX();
      isOnRamp = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {

    double currentPos = driveSubsystem.getOdometry().getPoseMeters().getX();
    double distanceTraveledOnRamp = currentPos - initialOnRampXPos;

    return isOnRamp && distanceTraveledOnRamp > metersToCenter;
  }
}
