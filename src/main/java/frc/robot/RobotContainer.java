// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.PrepareArmToPickUp;
import frc.robot.commands.PrepareArmToScore;
import frc.robot.commands.RunAutoCommand;
import frc.robot.commands.SetGripperModeToCone;
import frc.robot.commands.SetGripperModeToCube;
import frc.robot.commands.StowArm;
import frc.robot.controls.DualShock4Controller;
import frc.robot.controls.LogitechDualActionGamepad;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathFollowing;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  public enum GripperMode {
    Cone,
    Cube
  }

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
  // public static final ArmSubsystem armSubsystem = new ArmSubsystem();

  public static GripperMode gripperMode = GripperMode.Cone;

  private final LogitechDualActionGamepad pilot = new LogitechDualActionGamepad(0);
  private final LogitechDualActionGamepad copilot = new LogitechDualActionGamepad(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    initPilotControls();
    initCopilotControls();
    initAutoChooser();
    initDashboard();
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

  private void initCopilotControls() {
    copilot.lb.onTrue(new SetGripperModeToCone());
    copilot.rb.onTrue(new SetGripperModeToCube());

    // copilot.up.whileTrue(new RunCommand(() -> armSubsystem.increaseWristAngle()));
    // copilot.down.whileTrue(new RunCommand(() -> armSubsystem.decreaseWristAngle()));

    // copilot.l3.onTrue(new PrepareArmToPickUp());
    // copilot.r3.onTrue(new PrepareArmToScore());
    // copilot.b.onTrue(new StowArm());
  }

  private void initAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", new WaitCommand(3));
    autoChooser.addOption("Test", driveSubsystem.getPathPlannerCommand(PathFollowing.TestPath));
    autoChooser.addOption("Auto Target", new AutoTarget());
    autoChooser.addOption("U Turn Path", driveSubsystem.getPathPlannerCommand(PathFollowing.UTurnPath));
    autoChooser.addOption("Auto Balance", new AutoBalance());
    autoChooser.addOption("U Turn Path Copy", driveSubsystem.getPathPlannerCommand(PathFollowing.UTurnPathCopy));

    SmartDashboard.putData(autoChooser);
  }

  private void initDashboard() {

    var tab = Shuffleboard.getTab("Robot");

    tab.addString("Gripper Mode", () -> gripperMode.toString());

  }
}
