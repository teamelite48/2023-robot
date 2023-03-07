package frc.robot.subsystems.drive;

import java.util.HashMap;

import javax.management.InstanceAlreadyExistsException;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ReadyArm;
import frc.robot.commands.SetGripperModeToCone;
import frc.robot.commands.SetGripperModeToCube;
import frc.robot.commands.StowArm;

import frc.robot.subsystems.arm.ArmPreset;
import frc.robot.subsystems.arm.ArmPreset.DropZone;

public class PathFollowing {

    static public final PathPlannerTrajectory ScoreConeHoldCubeBalance = PathPlanner.loadPath("Score Cone Hold Cube Balance", new PathConstraints(2, 2));

    static public final HashMap<String, Command> EventMap = new HashMap<>() {{

        put("Cone Mode", new SetGripperModeToCone());
        put("Cube Mode", new SetGripperModeToCube());
        put("Pick Up Low", new ReadyArm(DropZone.Low, ArmPreset.PICK_UP_CONE_LOW, ArmPreset.PICK_UP_CUBE_LOW));
        put("Intake", new InstantCommand(() -> RobotContainer.gripperSubsystem.intake()));
        put("Score Mid", new ReadyArm(DropZone.Mid, ArmPreset.SCORE_CONE_MID, ArmPreset.SCORE_CUBE_MID));
        put("Outtake", new SequentialCommandGroup(
            new InstantCommand(() -> RobotContainer.gripperSubsystem.outtake()),
            new WaitCommand(1),
            new InstantCommand(() -> RobotContainer.gripperSubsystem.stop())
        ));
        put("Stop Intake", new InstantCommand(() -> RobotContainer.gripperSubsystem.stop()));
        put("Stow Arm", new StowArm());
        put("Auto Balance", new AutoBalance());

        put("Wait 1", new WaitCommand(1));
        put("Wait 2", new WaitCommand(2));
        put("Wait 3", new WaitCommand(3));
    }};
}
