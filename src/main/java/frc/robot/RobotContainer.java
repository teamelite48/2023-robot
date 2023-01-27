// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.controls.LogitechDualActionGamepad;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {

  private LogitechDualActionGamepad pilot = new LogitechDualActionGamepad(0);

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();

  public RobotContainer() {

    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.drive(
      pilot.getLeftXAxis(),
      pilot.getLeftYAxis(),
      pilot.getRightXAxis()
    ), driveSubsystem));

    initPilotControls();
  }

  private void initPilotControls() {

    pilot.x.onTrue(new InstantCommand(() -> logButtonPress("X")));

    pilot.a.whileTrue(new AutoBalanceCommand());

    pilot.b.onTrue(new InstantCommand(() -> logButtonPress("B")));
    pilot.y.onTrue(new InstantCommand(() -> logButtonPress("Y")));
    pilot.lb.onTrue(new InstantCommand(() -> logButtonPress("LB")));
    pilot.rb.onTrue(new InstantCommand(() -> logButtonPress("RB")));
    pilot.lt.onTrue(new InstantCommand(() -> logButtonPress("LT")));
    pilot.rt.onTrue(new InstantCommand(() -> logButtonPress("RT")));
    pilot.back.onTrue(new InstantCommand(() -> logButtonPress("Back")));

    pilot.start.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));

    pilot.l3.onTrue(new InstantCommand(() -> logButtonPress("L3")));
    pilot.r3.onTrue(new InstantCommand(() -> logButtonPress("R3")));
    pilot.up.onTrue(new InstantCommand(() -> logButtonPress("Up")));
    pilot.right.onTrue(new InstantCommand(() -> logButtonPress("Right")));
    pilot.down.onTrue(new InstantCommand(() -> logButtonPress("Down")));
    pilot.left.onTrue(new InstantCommand(() -> logButtonPress("Left")));
  }

  private void logButtonPress(String button) {
    System.out.println("Button " + button + " pressed");
  }

  public Command getAutonomousCommand() {
    return new InstantCommand(() -> {});
  }
}
