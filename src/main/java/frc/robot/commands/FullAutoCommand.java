// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.PathFollowing;

public class FullAutoCommand extends SequentialCommandGroup {

  public FullAutoCommand() {

    addCommands(
      RobotContainer.driveSubsystem.getFollowPathCommand(PathFollowing.UTurnPath, true),
      new AutoBalanceCommand()
    );
  }
}
