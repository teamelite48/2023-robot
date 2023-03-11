package frc.robot.subsystems.drive;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ReadyArm;
import frc.robot.commands.SetGripperModeToCone;
import frc.robot.commands.SetGripperModeToCube;
import frc.robot.commands.StowArm;

import frc.robot.subsystems.arm.ArmPreset;

public class PathFollowing {

    //static public final List<PathPlannerTrajectory> ScoreConeHoldCubeBalance = PathPlanner.loadPathGroup("Score Cone Hold Cube Balance", new PathConstraints(2, 2));
    static public final List<PathPlannerTrajectory> BarrierScoreConeBalance = PathPlanner.loadPathGroup("Barrier Score Cone High Balance", new PathConstraints(2.5, 2.5), new PathConstraints(2, 2));
    static public final List<PathPlannerTrajectory> WallScoreBalance = PathPlanner.loadPathGroup("Wall Score Balance", new PathConstraints(1.75, 1.75), new PathConstraints(1.5, 1.5), new PathConstraints(1, 1), new PathConstraints(2, 2), new PathConstraints(2, 2));
    static public final List<PathPlannerTrajectory> WallScoreConeScoreCube = PathPlanner.loadPathGroup("Wall Score Cone Score Cube", new PathConstraints(2, 2), new PathConstraints(1.5, 1.5), new PathConstraints(2, 2), new PathConstraints(1.5, 1.5), new PathConstraints(2, 2));
    static public final List<PathPlannerTrajectory> BarrierScoreConeScoreCube = PathPlanner.loadPathGroup("Barrier Score Cone Score Cube", new PathConstraints(2.25, 2.25));
    //static public final List<PathPlannerTrajectory> ScoreConeScoreCube = PathPlanner.loadPathGroup("Score Cone Score Cube", new PathConstraints(2, 2));
    static public final List<PathPlannerTrajectory> MiddleScoreConeBalance = PathPlanner.loadPathGroup("Middle Score Cone High Balance", new PathConstraints(2, 2));
    //static public final List<PathPlannerTrajectory> Test = PathPlanner.loadPathGroup("New Path", new PathConstraints(2, 2));

    //static public final List<PathPlannerTrajectory> Test = PathPlanner.loadPathGroup("Test", new PathConstraints(1, 1), new PathConstraints(3, 3));

    static public final HashMap<String, Command> EventMap = new HashMap<>() {{

        put("Cone Mode", new SetGripperModeToCone());
        put("Cube Mode", new SetGripperModeToCube());
        put("Pick Up Low", new ReadyArm(ArmPreset.PICK_UP_CONE_LOW, ArmPreset.PICK_UP_CUBE_LOW));
        put("Intake", new InstantCommand(() -> RobotContainer.gripperSubsystem.intake()));
        put("Score Mid", new ReadyArm(ArmPreset.SCORE_CONE_MID, ArmPreset.SCORE_CUBE_MID));
        put("Score High", new ReadyArm(ArmPreset.SCORE_CONE_HIGH, ArmPreset.SCORE_CUBE_HIGH));
        put("Outtake", new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.gripperSubsystem.outtake()),
            new WaitCommand(.75),
            new InstantCommand(() -> RobotContainer.gripperSubsystem.stop())
        ));
        put("Stop Intake", new InstantCommand(() -> RobotContainer.gripperSubsystem.stop()));
        put("Stow Arm", new StowArm());
        put("Auto Balance", new AutoBalance());
        put("Zero Gyro", new InstantCommand(() -> RobotContainer.driveSubsystem.zeroGyro()));

        put("Wait 0.5", new WaitCommand(0.5));
        put("Wait 1", new WaitCommand(1));
        put("Wait 2", new WaitCommand(2));
        put("Wait 3", new WaitCommand(3));
    }};
}
