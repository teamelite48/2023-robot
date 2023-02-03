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

  private long balancedMillis;

  public AutoBalanceCommand() {
    driveSubsystem = RobotContainer.driveSubsystem;
    addRequirements(RobotContainer.driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.zeroGyro();
    speed = 0.15;
    desiredAngle = 2.0;
    lastState = BALANCED;
    balancedMillis = 0;
  }

  @Override
  public void execute() {

    currentState = getState();

    if (currentState != lastState && speed >= 0.0) {
      speed = speed - 0.02;
    }

    if (currentState == PITCH_UP) {
      driveForward();
      balancedMillis = 0;
    }
    else if (currentState == PITCH_DOWN) {
       driveBackward();
       balancedMillis = 0;
    }
    else {
      stop();
      balancedMillis = System.currentTimeMillis();
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
    return currentState == BALANCED && (System.currentTimeMillis() - balancedMillis) > 2000;
  }

  private void driveForward() {
    driveSubsystem.drive(0, -speed, 0, false);
  }

  private void driveBackward() {
    driveSubsystem.drive(0, speed, 0, false);
  }

  private void stop() {
    driveSubsystem.drive(0, 0, 0, false);
  }
}

