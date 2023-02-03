// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveSubsystem;


public class FullAutoCommand extends SequentialCommandGroup {

  public FullAutoCommand() {
    DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;

    var path = PathPlanner.loadPath("U Turn Path Copy", new PathConstraints(1, 1));
    var pathCommand = driveSubsystem.followTrajectoryCommand(path, true);

    addCommands(
      pathCommand,
      new AutoBalanceCommand()
    );
  }
}
