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
import frc.robot.controls.LogitechDualActionGamepad;
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

  private final LogitechDualActionGamepad pilotController = new LogitechDualActionGamepad(0);
  private final LogitechDualActionGamepad copilotController = new LogitechDualActionGamepad(1);
  private final LogitechDualActionGamepad testController = new LogitechDualActionGamepad(2);

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

    pilotController.lb.onTrue(new SetGripperModeToCone());
    pilotController.rb.onTrue(new SetGripperModeToCube());

    pilotController.lt
      .whileTrue(new InstantCommand(() -> gripperSubsystem.intake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    pilotController.rt
      .whileTrue(new InstantCommand(() -> gripperSubsystem.outtake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    pilotController.a.onTrue(new ReadyArm(ArmPreset.PICK_UP_CONE_LOW, ArmPreset.PICK_UP_CUBE_LOW));
    pilotController.x.onTrue(new ReadyArm(ArmPreset.PICK_UP_CONE_MID, ArmPreset.PICK_UP_CUBE_MID));
    pilotController.y.onTrue(new ReadyArm(ArmPreset.PICK_UP_CONE_HIGH, ArmPreset.PICK_UP_CUBE_HIGH));
    pilotController.b.onTrue(new StowArm());

    pilotController.back.whileTrue(new RunAutoCommand(() -> autoChooser.getSelected()));
    pilotController.start.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));

    pilotController.l3.onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));
    pilotController.r3.onTrue(new InstantCommand(() -> driveSubsystem.setHighGear()));
  }

  private void initCopilotController() {

    armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.manualPosition(
      -copilotController.getLeftYAxis(),
      -copilotController.getRightYAxis()),
      armSubsystem
    ));

    copilotController.lb.onTrue(new SetGripperModeToCone());
    copilotController.rb.onTrue(new SetGripperModeToCube());

    copilotController.a.onTrue(new ReadyArm(ArmPreset.SCORE_CONE_LOW, ArmPreset.SCORE_CUBE_LOW));
    copilotController.x.onTrue(new ReadyArm(ArmPreset.SCORE_CONE_MID, ArmPreset.SCORE_CUBE_MID));
    copilotController.y.onTrue(new ReadyArm(ArmPreset.SCORE_CONE_HIGH, ArmPreset.SCORE_CUBE_HIGH));
    copilotController.b.onTrue(new StowArm());

    copilotController.lt
      .whileTrue(new InstantCommand(() -> gripperSubsystem.intake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    copilotController.rt
      .whileTrue(new InstantCommand(() -> gripperSubsystem.outtake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));
  }


  private void initTestController() {
    testController.up.onTrue(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(ArmConfig.JOINT_MOTOR_TEST_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(0)));

    testController.down.onTrue(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(-ArmConfig.JOINT_MOTOR_TEST_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(0)));

    testController.lb.onTrue(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(ArmConfig.JOINT_MOTOR_TEST_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(0)));

    testController.rb.onTrue(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(-ArmConfig.JOINT_MOTOR_TEST_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(0)));

    testController.lt.onTrue(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(ArmConfig.JOINT_MOTOR_TEST_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(0)));

    testController.rt.onTrue(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(-ArmConfig.JOINT_MOTOR_TEST_SPEED)))
      .onFalse(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(0)));
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
