// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {

  private final int BALANCED = 0;
  private final int PITCH_UP = 1;
  private final int PITCH_DOWN = 2;

  private DriveSubsystem driveSubsystem;

  private int lastState;
  private int currentState;
  private double speed;
  private double desiredAngle;


  public AutoBalanceCommand() {
    driveSubsystem = RobotContainer.driveSubsystem;
    addRequirements(RobotContainer.driveSubsystem);
  }

  @Override
  public void initialize() {
    speed = 0.4;
    desiredAngle = 1.0;
    lastState = BALANCED;
  }

  @Override
  public void execute() {

    currentState = getState();

    if (currentState != lastState && speed >= 0.0) {
      speed = speed - 0.05;
    }

    if (currentState == PITCH_UP) {
      driveForward();
    }
    else if (currentState == PITCH_DOWN) {
       driveBackward();
    }
    else {
      stop();
    }

    lastState = currentState;
  }

  private int getState() {

    if (driveSubsystem.getPitch() > desiredAngle) {
      return PITCH_UP;
    }
    else if (driveSubsystem.getPitch() < -desiredAngle) {
      return PITCH_DOWN;
    }
    else {
      return BALANCED;
    }
  }

  @Override
  public void end(boolean interrupted) {
    stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void driveForward() {
    driveSubsystem.drive(0, -speed, 0);
  }

  private void driveBackward() {
    driveSubsystem.drive(0, speed, 0);
  }

  private void stop() {
    driveSubsystem.drive(0, 0, 0);
  }
}

