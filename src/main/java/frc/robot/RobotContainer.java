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
import frc.robot.controls.LogitechDualActionGamepad;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathFollowing;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {



  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final GripperSubsystem gripperSubsystem = new GripperSubsystem();



  private final LogitechDualActionGamepad pilot = new LogitechDualActionGamepad(0);
  private final LogitechDualActionGamepad copilot = new LogitechDualActionGamepad(1);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    initPilotControls();
    initCopilotControls();
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

    pilot.a.onTrue(new InstantCommand(() -> armSubsystem.setArmState(ArmState.PickUpLow)));
    pilot.x.onTrue(new InstantCommand(() -> armSubsystem.setArmState(ArmState.PickUpMid)));
    pilot.y.onTrue(new InstantCommand(() -> armSubsystem.setArmState(ArmState.PickUpHigh)));
    pilot.b.onTrue(new StowArm());

    pilot.back.whileTrue(new RunAutoCommand(() -> autoChooser.getSelected()));
    pilot.start.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));

    pilot.l3.onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));
    pilot.r3.onTrue(new InstantCommand(() -> driveSubsystem.setHighGear()));
  }

  private void initCopilotControls() {

    copilot.lb.onTrue(new SetGripperModeToCone());
    copilot.rb.onTrue(new SetGripperModeToCube());

    copilot.up.whileTrue(new RunCommand(() -> armSubsystem.increaseWristAngle()));
    copilot.down.whileTrue(new RunCommand(() -> armSubsystem.decreaseWristAngle()));

    copilot.a.onTrue(new InstantCommand(() -> armSubsystem.setArmState(ArmState.ScoreLow)));
    copilot.x.onTrue(new InstantCommand(() -> armSubsystem.setArmState(ArmState.ScoreMid)));
    copilot.y.onTrue(new InstantCommand(() -> armSubsystem.setArmState(ArmState.ScoreHigh)));

    copilot.r3.onTrue(new PrepareArmToScore());


    copilot.b.onTrue(new StowArm());

    copilot.lt
      .whileTrue(new InstantCommand(() -> gripperSubsystem.intake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    copilot.rt
      .whileTrue(new InstantCommand(() -> gripperSubsystem.outtake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));
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


}
