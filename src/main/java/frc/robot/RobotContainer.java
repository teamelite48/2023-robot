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
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.ReadyArm;
import frc.robot.commands.RunAutoCommand;
import frc.robot.commands.SetGripperModeToCone;
import frc.robot.commands.SetGripperModeToCube;
import frc.robot.commands.StowArm;
import frc.robot.controls.DualShock4Controller;
import frc.robot.subsystems.arm.ArmConfig;
import frc.robot.subsystems.arm.ArmPreset;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PathFollowing;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {

  public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static final VisionSubsystem visionSubsystem = new VisionSubsystem();
  public static final ArmSubsystem armSubsystem = new ArmSubsystem();
  public static final GripperSubsystem gripperSubsystem = new GripperSubsystem();
  public static final LedSubsystem ledSubsystem = new LedSubsystem();

  private final DualShock4Controller pilotController = new DualShock4Controller(0);
  private final DualShock4Controller copilotController = new DualShock4Controller(1);
  private final DualShock4Controller testController = new DualShock4Controller(2);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    initPilotController();
    initCopilotController();
    initTestController();

    initAutoChooser();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void initPilotController() {

    driveSubsystem.setDefaultCommand(new RunCommand(() -> driveSubsystem.manualDrive(
      pilotController.getLeftXAxis(),
      pilotController.getLeftYAxis(),
      pilotController.getRightXAxis()
    ), driveSubsystem));

    pilotController.l1.onTrue(new SetGripperModeToCone());
    pilotController.r1.onTrue(new SetGripperModeToCube());

    pilotController.l2
      .whileTrue(new InstantCommand(() -> gripperSubsystem.intake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    pilotController.r2
      .whileTrue(new InstantCommand(() -> gripperSubsystem.outtake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    // pilotController.cross.onTrue(new ReadyArm(ArmPreset.PICK_UP_CONE_LOW, ArmPreset.PICK_UP_CUBE_LOW));
    // pilotController.square.onTrue(new ReadyArm(ArmPreset.PICK_UP_CONE_MID, ArmPreset.PICK_UP_CUBE_MID));
    // pilotController.triangle.onTrue(new ReadyArm(ArmPreset.PICK_UP_CONE_HIGH, ArmPreset.PICK_UP_CUBE_HIGH));
    // pilotController.circle.onTrue(new StowArm());

    pilotController.share.whileTrue(new RunAutoCommand(() -> autoChooser.getSelected()));
    pilotController.options.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));

    pilotController.l3.onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));
    pilotController.r3.onTrue(new InstantCommand(() -> driveSubsystem.setHighGear()));
  }

  private void initCopilotController() {

    // armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.manualPosition(
    //   -copilotController.getLeftYAxis(),
    //   -copilotController.getRightYAxis()),
    //   armSubsystem
    // ));

    copilotController.l1.onTrue(new SetGripperModeToCone());
    copilotController.r1.onTrue(new SetGripperModeToCube());

    // copilotController.cross.onTrue(new ReadyArm(ArmPreset.SCORE_CONE_LOW, ArmPreset.SCORE_CUBE_LOW));
    // copilotController.square.onTrue(new ReadyArm(ArmPreset.SCORE_CONE_MID, ArmPreset.SCORE_CUBE_MID));
    // copilotController.triangle.onTrue(new ReadyArm(ArmPreset.SCORE_CONE_HIGH, ArmPreset.SCORE_CUBE_HIGH));
    // copilotController.circle.onTrue(new StowArm());

    copilotController.l2
      .whileTrue(new InstantCommand(() -> gripperSubsystem.intake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    copilotController.r2
      .whileTrue(new InstantCommand(() -> gripperSubsystem.outtake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));
  }


  private void initTestController() {

    testController.up.onTrue(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(ArmConfig.SHOULDER_MAX_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(0)));

    testController.down.onTrue(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(-ArmConfig.SHOULDER_MAX_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(0)));

    testController.l1.onTrue(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(-ArmConfig.ELBOW_MAX_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(0)));

    testController.r1.onTrue(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(ArmConfig.ELBOW_MAX_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(0)));

    testController.cross.onTrue(new ReadyArm(ArmPreset.DROP_ZONE, ArmPreset.DROP_ZONE));
    testController.circle.onTrue(new StowArm());

    // testController.l2.whileTrue(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(ArmConfig.WRIST_TEST_SPEED)))
    //   .onFalse(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(0)));

    // testController.r2.onTrue(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(-ArmConfig.WRIST_TEST_SPEED)))
    //   .onFalse(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(0)));
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
