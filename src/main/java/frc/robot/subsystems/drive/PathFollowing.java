package frc.robot.subsystems.drive;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoTargetCommand;

public class PathFollowing {

    static public final PathPlannerTrajectory TestPath = PathPlanner.loadPath("Test Path", new PathConstraints(2, 2));
    static public final PathPlannerTrajectory UTurnPath = PathPlanner.loadPath("U Turn Path", new PathConstraints(2, 2));
    static public final PathPlannerTrajectory UTurnCopyPath = PathPlanner.loadPath("U Turn Path Copy", new PathConstraints(2, 2));
    static public final PathPlannerTrajectory ConeScore1Hold1Balance = PathPlanner.loadPath("Cone Score 1 Hold 1 Balance", new PathConstraints(2, 2));

    static public final HashMap<String, Command> EventMap = new HashMap<>() {{
        put("Auto Balance", new AutoBalanceCommand());
        put("Auto Target", new AutoTargetCommand());
        put("Enable AprilTag Pipeline", new InstantCommand(() -> RobotContainer.visionSubsystem.enableAprilTagPipeline()));
        put("Enable Reflective Tape Pipeline", new InstantCommand(() -> RobotContainer.visionSubsystem.enableReflectiveTapePipeline()));
    }};
}
