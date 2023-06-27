// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoTarget;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ReadyArm;
import frc.robot.commands.SetGripperModeToCone;
import frc.robot.commands.SetGripperModeToCube;
import frc.robot.commands.ToggleArmOrientation;
import frc.robot.controls.DualShock4Controller;
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

  private final DualShock4Controller pilotController = new DualShock4Controller(0);
  private final DualShock4Controller copilotController = new DualShock4Controller(1);
  private final LogitechDualActionGamepad testController = new LogitechDualActionGamepad(2);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {

    initPilotController();
    initCopilotController();
    initTestController();

    initAutoChooser();
    //initCamera();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void initPilotController() {

    driveSubsystem.setDefaultCommand(new DriveCommand(() -> pilotController.getLeftAxes(), () -> pilotController.getRightAxes()));

    pilotController.ps.onTrue(new ToggleArmOrientation());

    pilotController.l1.onTrue(new SetGripperModeToCone());
    pilotController.r1.onTrue(new SetGripperModeToCube());

    pilotController.l2
      .whileTrue(new InstantCommand(() -> gripperSubsystem.intake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.hold()));

    pilotController.r2
      .whileTrue(new InstantCommand(() -> gripperSubsystem.outtake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    pilotController.share.whileTrue(new AutoBalance());
    pilotController.options.onTrue(new InstantCommand(() -> driveSubsystem.zeroGyro()));

    pilotController.l3.onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));
    pilotController.r3.onTrue(new InstantCommand(() -> driveSubsystem.setHighGear()));

    pilotController.cross
      .onTrue(new SequentialCommandGroup(
        new ReadyArm(ArmPreset.PICK_UP_CONE_LOW, ArmPreset.PICK_UP_CUBE_LOW),
        new InstantCommand(() -> driveSubsystem.setLowGear())
      ));


    pilotController.square
      .onTrue(new SequentialCommandGroup(
        new ReadyArm(ArmPreset.PICK_UP_CONE_MID, ArmPreset.PICK_UP_CUBE_MID),
        new InstantCommand(() -> driveSubsystem.setLowGear())
      ));


    pilotController.triangle
      .onTrue(new SequentialCommandGroup(
        new ReadyArm(ArmPreset.PICK_UP_CONE_HIGH, ArmPreset.PICK_UP_CUBE_HIGH),
        new InstantCommand(() -> driveSubsystem.setLowGear())
      ));

    pilotController.circle
      .onTrue(new ReadyArm(ArmPreset.STOWED_CONE, ArmPreset.STOWED_CUBE))
      .onTrue(new InstantCommand(() -> driveSubsystem.setHighGear()));
    }

  private void initCopilotController() {

    armSubsystem.setDefaultCommand(new RunCommand(() -> armSubsystem.manualPosition(
      -copilotController.getLeftAxes().getY(),
      -copilotController.getRightAxes().getY()),
      armSubsystem
    ));

    copilotController.ps.onTrue(new ToggleArmOrientation());

    copilotController.l1.onTrue(new SetGripperModeToCone());
    copilotController.r1.onTrue(new SetGripperModeToCube());

    copilotController.l2
      .onTrue(new InstantCommand(() -> gripperSubsystem.intake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.hold()));

    copilotController.r2
      .onTrue(new InstantCommand(() -> gripperSubsystem.outtake()))
      .onFalse(new InstantCommand(() -> gripperSubsystem.stop()));

    copilotController.share.whileTrue(new AutoTarget());

    copilotController.cross
      .onTrue(new ReadyArm(ArmPreset.SCORE_CONE_LOW, ArmPreset.SCORE_CUBE_LOW))
      .onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));

    copilotController.square
      .onTrue(new ReadyArm(ArmPreset.SCORE_CONE_MID, ArmPreset.SCORE_CUBE_MID))
      .onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));

    copilotController.triangle
      .onTrue(new ReadyArm(ArmPreset.SCORE_CONE_HIGH, ArmPreset.SCORE_CUBE_HIGH))
      .onTrue(new InstantCommand(() -> driveSubsystem.setLowGear()));

    copilotController.circle
      .onTrue(new ReadyArm(ArmPreset.STOWED_CONE, ArmPreset.STOWED_CONE))
      .onTrue(new InstantCommand(() -> driveSubsystem.setHighGear()));
  }

  private void initTestController() {

    testController.up.onTrue(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(ArmConfig.SHOULDER_MAX_SPEED * ArmConfig.TEST_MAX_SPEED_MODIFIER)))
      .onFalse(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(0)));

    testController.down.onTrue(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(-ArmConfig.SHOULDER_MAX_SPEED * ArmConfig.TEST_MAX_SPEED_MODIFIER)))
      .onFalse(new InstantCommand(() -> armSubsystem.setShoulderMotorSpeed(0)));

    testController.lb.onTrue(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(-ArmConfig.ELBOW_MAX_SPEED * ArmConfig.TEST_MAX_SPEED_MODIFIER)))
      .onFalse(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(0)));

    testController.rb.onTrue(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(ArmConfig.ELBOW_MAX_SPEED * ArmConfig.TEST_MAX_SPEED_MODIFIER)))
      .onFalse(new InstantCommand(() -> armSubsystem.setElbowMotorSpeed(0)));

    testController.lt.onTrue(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(-ArmConfig.WRIST_MAX_SPEED * ArmConfig.TEST_MAX_SPEED_MODIFIER)))
      .onFalse(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(0)));

    testController.rt.onTrue(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(ArmConfig.WRIST_MAX_SPEED * ArmConfig.TEST_MAX_SPEED_MODIFIER)))
      .onFalse(new InstantCommand(() -> armSubsystem.setWristMotorSpeed(0)));

    testController.back.whileTrue(new AutoBalance());
  }

  private void initAutoChooser() {
    autoChooser.setDefaultOption("GLHF (Do Nothing)", new WaitCommand(3));

    autoChooser.addOption("Barrier SCORE Cone HOLD Cube BALANCE", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.BarrierScoreConeHoldCubeBalance));
    autoChooser.addOption("Barrier SCORE Cone SCORE Cube", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.BarrierScoreConeScoreCube));
    autoChooser.addOption("Barrier SCORE Cone SCORE Cube BALANCE", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.BarrierScoreConeScoreCubeBalance));
    autoChooser.addOption("Barrier SCORE 3 Low", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.BarrierLowThreePiece));

    autoChooser.addOption("Wall SCORE Cone HOLD Cube BALANCE", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.WallScoreConeHoldCubeBalance));
    autoChooser.addOption("Wall SCORE Cone SCORE Cube", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.WallScoreConeScoreCube));
    autoChooser.addOption("Wall SCORE Cone SCORE Cube BALANCE", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.WallScoreConeScoreCubeBalance));

    autoChooser.addOption("Middle Barrier SCORE Cone HOLD Cube BALANCE", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.BarrierMiddleScoreConeHoldCubeBalance));
    autoChooser.addOption("Middle Wall SCORE Cone HOLD Cube BALANCE", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.WallMiddleScoreConeHoldCubeBalance));
    autoChooser.addOption("Middle Barrier SCORE Cone BALANCE", driveSubsystem.getPathPlannerGroupCommand(PathFollowing.MiddleScoreConeBalance));

    SmartDashboard.putData(autoChooser);
  }

  // private void initCamera(){

  //   if (RobotBase.isSimulation()) return;

  //   UsbCamera camera = CameraServer.startAutomaticCapture();
  //   camera.setResolution(160, 120);
  //   camera.setFPS(20);
  //   camera.setExposureAuto();
  // }
}
