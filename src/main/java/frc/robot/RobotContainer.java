// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.controls.LogitechDualActionGamepad;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
 
  private LogitechDualActionGamepad pilotGamepad = new LogitechDualActionGamepad(0, 0.02, true, 0.25);

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {

    Command driveCommand = new RunCommand(() -> driveSubsystem.drive(
      pilotGamepad.getLeftYAxis(),
      pilotGamepad.getLeftXAxis(),
      pilotGamepad.getRightXAxis(),
      true
    ), driveSubsystem);

    driveSubsystem.setDefaultCommand(driveCommand);
      
    initPilotControls();  
  }

  private void initPilotControls() {

    pilotGamepad.x.onTrue(new InstantCommand(() -> logButtonPress("X")));
    pilotGamepad.a.onTrue(new InstantCommand(() -> logButtonPress("A")));
    pilotGamepad.b.onTrue(new InstantCommand(() -> logButtonPress("B")));
    pilotGamepad.y.onTrue(new InstantCommand(() -> logButtonPress("Y")));
    pilotGamepad.lb.onTrue(new InstantCommand(() -> logButtonPress("LB")));
    pilotGamepad.rb.onTrue(new InstantCommand(() -> logButtonPress("RB")));
    pilotGamepad.lt.onTrue(new InstantCommand(() -> logButtonPress("LT")));
    pilotGamepad.rt.onTrue(new InstantCommand(() -> logButtonPress("RT")));
    pilotGamepad.back.onTrue(new InstantCommand(() -> logButtonPress("Back")));

    pilotGamepad.start.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));
    
    pilotGamepad.l3.onTrue(new InstantCommand(() -> logButtonPress("L3")));
    pilotGamepad.r3.onTrue(new InstantCommand(() -> logButtonPress("R3")));
    pilotGamepad.up.onTrue(new InstantCommand(() -> logButtonPress("Up")));
    pilotGamepad.right.onTrue(new InstantCommand(() -> logButtonPress("Right")));
    pilotGamepad.down.onTrue(new InstantCommand(() -> logButtonPress("Down")));
    pilotGamepad.left.onTrue(new InstantCommand(() -> logButtonPress("Left")));
  }

  private void logButtonPress(String button) {
    System.out.println("Button " + button + " pressed");
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(() -> {});
  }
}
