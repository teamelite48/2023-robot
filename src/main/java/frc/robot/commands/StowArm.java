// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.ArmSubsystem.ArmState;


public class StowArm extends SequentialCommandGroup {



  public StowArm() {

    // var armSubsystem = RobotContainer.armSubsystem;

    // addCommands(
    //   new InstantCommand(() -> armSubsystem.setArmState(ArmState.Tansitioning), armSubsystem),
    //   new WaitCommand(3),
    //   new InstantCommand(() -> armSubsystem.setArmState(ArmState.Stowed), armSubsystem)
    // );
  }
}
