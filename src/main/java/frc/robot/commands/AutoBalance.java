// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalance extends CommandBase {

  enum Pitch {
    Balanced,
    Up,
    Down
  }

  private DriveSubsystem driveSubsystem;

  private Pitch previousPitch;
  private Pitch currentPitch;
  private double speed;
  private double desiredAngle;
  private long balancedMillis;

  public AutoBalance() {
    driveSubsystem = RobotContainer.driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.zeroGyro();
    speed = 0.125;
    desiredAngle = 2.5;
    previousPitch = null;
    balancedMillis = 0;
  }

  @Override
  public void execute() {

    currentPitch = getCurrentPitch();

    if (previousPitch != null && currentPitch != previousPitch && speed >= 0.0) {
      speed -= 0.015;
    }

    if (currentPitch == Pitch.Up) {
      driveForward();
      balancedMillis = 0;
    }
    else if (currentPitch == Pitch.Down) {
       driveBackward();
       balancedMillis = 0;
    }
    else {
      stop();

      if (balancedMillis == 0) {
        balancedMillis = System.currentTimeMillis();
      }
    }

    previousPitch = currentPitch;
  }

  @Override
  public void end(boolean interrupted) {
    stop();
  }

  @Override
  public boolean isFinished() {

    if (currentPitch == Pitch.Balanced) {
      var balancedDuration = System.currentTimeMillis() - balancedMillis;
      return balancedDuration > 2000;
    }

    return false;
  }

  private Pitch getCurrentPitch() {

    if (driveSubsystem.getPitch() > desiredAngle) {
      return Pitch.Up;
    }
    else if (driveSubsystem.getPitch() < -desiredAngle) {
      return Pitch.Down;
    }
    else {
      return Pitch.Balanced;
    }
  }

  private void driveForward() {
    driveSubsystem.autoDrive(0, -speed, 0);
  }

  private void driveBackward() {
    driveSubsystem.autoDrive(0, speed, 0);
  }

  private void stop() {
    driveSubsystem.autoDrive(0, 0, 0);
  }
}