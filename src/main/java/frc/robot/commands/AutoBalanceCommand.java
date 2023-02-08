// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {

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

  public AutoBalanceCommand() {
    driveSubsystem = RobotContainer.driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.zeroGyro();
    speed = 0.15;
    desiredAngle = 2.0;
    previousPitch = null;
    balancedMillis = 0;
  }

  @Override
  public void execute() {

    currentPitch = getCurrentPitch();

    if (previousPitch != null && currentPitch != previousPitch && speed >= 0.0) {
      speed -= 0.02;
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
    var balancedDuration = System.currentTimeMillis() - balancedMillis;
    return currentPitch == Pitch.Balanced && balancedDuration > 2000;
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