// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoTargetCommand;
import frc.robot.commands.FullAutoCommand;
import frc.robot.commands.RunAutoCommand;
import frc.robot.controls.LogitechDualActionGamepad;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathFollowing;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final LogitechDualActionGamepad pilot = new LogitechDualActionGamepad(0);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    initPilotControls();
    initAutoChooser();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void initPilotControls() {

    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.manualDrive(
      pilot.getLeftXAxis(),
      pilot.getLeftYAxis(),
      pilot.getRightXAxis()
    ), driveSubsystem));

    pilot.back.whileTrue(new RunAutoCommand(() -> autoChooser.getSelected()));
    pilot.start.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));

    pilot.l3.onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));
    pilot.r3.onTrue(new InstantCommand(() -> driveSubsystem.setHighGear()));
  }

  private void initAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(3));
    autoChooser.addOption("Test", driveSubsystem.getFollowPathCommand(PathFollowing.TestPath, true));
    autoChooser.addOption("Auto Target", new AutoTargetCommand());
    autoChooser.addOption("Full Auto", new FullAutoCommand());
    autoChooser.addOption("Cone Score 1 Hold 1 Balance", driveSubsystem.getFollowPathWithEventsCommand(PathFollowing.ConeScore1Hold1Balance));

    SmartDashboard.putData(autoChooser);
  }
}
