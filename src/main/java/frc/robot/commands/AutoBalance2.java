// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalance2 extends CommandBase {

  DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

  final double INITIAL_SPEED = 0.25;
  final double METERS_TO_CENTER = 0.535;
  final double ON_RAMP_PITCH_FLOOR = 13.5;
  final double ON_RAMP_PITCH_CEILING = 14.0;

  double initialRampPosition = 0.0;
  boolean isOnRamp = false;
  double speed;

  public AutoBalance2() {
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    speed = INITIAL_SPEED;
    isOnRamp = false;
    initialRampPosition = 0.0;
  }

  @Override
  public void execute() {

    driveSubsystem.autoDrive(0, -speed, 0);

    var currentPitch = Math.abs(driveSubsystem.getPitch());

    if (isOnRamp == false && ON_RAMP_PITCH_FLOOR < currentPitch && currentPitch < ON_RAMP_PITCH_CEILING) {
      initialRampPosition = driveSubsystem.getOdometry().getPoseMeters().getX();
      isOnRamp = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.autoDrive(0, 0, 0);
  }

  @Override
  public boolean isFinished() {

    if (isOnRamp == false) {
      return false;
    }

    double currentPosition = driveSubsystem.getOdometry().getPoseMeters().getX();
    double distanceTraveled = Math.abs(currentPosition - initialRampPosition);

    return distanceTraveled > METERS_TO_CENTER;
  }
}
