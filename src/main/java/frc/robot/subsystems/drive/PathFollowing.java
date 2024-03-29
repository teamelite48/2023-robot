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
import frc.robot.commands.AutoStow;
import frc.robot.commands.ReadyArm;
import frc.robot.commands.SetGripperModeToCone;
import frc.robot.commands.SetGripperModeToCube;
import frc.robot.commands.StowArm;
import frc.robot.commands.ToggleArmOrientation;
import frc.robot.commands.WaitForArmToBeInsideFramePerimeter;
import frc.robot.commands.WaitUntilArmIsReady;
import frc.robot.subsystems.arm.ArmPreset;

public class PathFollowing {

    static public final List<PathPlannerTrajectory> BarrierScoreConeHoldCubeBalance = PathPlanner.loadPathGroup("Barrier Score Cone Hold Cube Balance", new PathConstraints(2, 2));
    static public final List<PathPlannerTrajectory> BarrierScoreConeScoreCube = PathPlanner.loadPathGroup("Barrier Score Cone Score Cube", new PathConstraints(2, 2), new PathConstraints(2, 2), new PathConstraints(3.5, 3.5));
    static public final List<PathPlannerTrajectory> BarrierScoreConeScoreCubeBalance = PathPlanner.loadPathGroup("Barrier Score Cone Score Cube Balance", new PathConstraints(2.5, 2.5), new PathConstraints(3, 3), new PathConstraints(2.75, 2.75));
    static public final List<PathPlannerTrajectory> BarrierLowThreePiece = PathPlanner.loadPathGroup("Barrier Low 3 Piece", new PathConstraints(2.25, 2.25));
    static public final List<PathPlannerTrajectory> WallScoreConeHoldCubeBalance = PathPlanner.loadPathGroup("Wall Score Cone Hold Cube Balance", new PathConstraints(2.25, 2.25));
    static public final List<PathPlannerTrajectory> WallScoreConeScoreCube = PathPlanner.loadPathGroup("Wall 2 Score Redo", new PathConstraints(2.25, 2.25));
    static public final List<PathPlannerTrajectory> WallScoreConeScoreCubeBalance = PathPlanner.loadPathGroup("Wall Score Cone Score Cube Balance", new PathConstraints(2.5, 2.5), new PathConstraints(3, 3), new PathConstraints(2.5, 2.5));
    static public final List<PathPlannerTrajectory> BarrierMiddleScoreConeHoldCubeBalance = PathPlanner.loadPathGroup("The Middle Barrier Redo", new PathConstraints(1.25, 1.25), new PathConstraints(2.3, 2.3));
    static public final List<PathPlannerTrajectory> WallMiddleScoreConeHoldCubeBalance = PathPlanner.loadPathGroup("Wall Middle Score Hold Cube Balance", new PathConstraints(1.25, 1.25), new PathConstraints(2.3, 2.3));
    static public final List<PathPlannerTrajectory> MiddleScoreConeBalance = PathPlanner.loadPathGroup("Middle Score Cone Balance", new PathConstraints(1.25, 1.25), new PathConstraints(2.2, 2.2));


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
            new WaitCommand(.35),
            new InstantCommand(() -> RobotContainer.gripperSubsystem.stop())
        ));
        put("OuttakeSlower", new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.gripperSubsystem.outtakeSlower()),
            new WaitCommand(.35),
            new InstantCommand(() -> RobotContainer.gripperSubsystem.stop())
        ));
        put("Stop Intake", new InstantCommand(() -> RobotContainer.gripperSubsystem.stop()));
        put("Stow Arm", new StowArm());
        put("Auto Stow", new AutoStow());
        put("Toggle Arm Orientation", new ToggleArmOrientation());
        put("Wait For Arm To Be Inside Frame Perimeter", new WaitForArmToBeInsideFramePerimeter());
        put("Arm Ready", new WaitUntilArmIsReady());
        put("Auto Balance", new AutoBalance());
        put("Zero Gyro", new InstantCommand(() -> RobotContainer.driveSubsystem.zeroGyro()));

        put("Wait 0.3", new WaitCommand(0.3));
        put("Wait 0.5", new WaitCommand(0.5));
        put("Wait 1", new WaitCommand(1));
        put("Wait 1.5", new WaitCommand(1.5));
        put("Wait 2", new WaitCommand(2));
        put("Wait 3", new WaitCommand(3));
    }};
}
