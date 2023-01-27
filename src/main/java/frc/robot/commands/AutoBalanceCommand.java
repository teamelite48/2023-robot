// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoBalanceCommand extends CommandBase {

  private DriveSubsystem driveSubsystem;
  private boolean isOnRamp;
  private double balanceSpeed;

  public AutoBalanceCommand() {
    driveSubsystem = RobotContainer.driveSubsystem;
    addRequirements(RobotContainer.driveSubsystem);
  }

  @Override
  public void initialize() {
    isOnRamp = false;
    balanceSpeed = 0.75;
  }

  @Override
  public void execute() {

    if (driveSubsystem.getPitch() < 10 && isOnRamp == false) {
      driveSubsystem.drive(0, -balanceSpeed, 0);
    }
    else if (driveSubsystem.getPitch() > 7) {
      isOnRamp = true;
      driveSubsystem.drive(0, -balanceSpeed, 0);

    }
    else if (driveSubsystem.getPitch() < -7) {
      balanceSpeed = 0.35;
      driveSubsystem.drive(0, balanceSpeed, 0);
    }
    else {
      stop();
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

  // private void driveForward() {
  //   driveSubsystem.drive(0, -balancedSpeed, 0);
  // }

  // private void driveBackward() {
  //   driveSubsystem.drive(0, balancedSpeed, 0);
  // }

  private void stop() {
    driveSubsystem.drive(0, 0, 0);
  }
}

