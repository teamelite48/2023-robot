package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.ArmSubsystem.ArmState;

public class ArmPreset {

    public static final ArmPosition STOWED = new ArmPosition(0.38, 0.16, 90, ArmState.Stowed);

    public static final ArmPosition DROP_ZONE = new ArmPosition(0.85, 0.16, 0.0, ArmState.Ready);

    public static final ArmPosition PICK_UP_CONE_LOW= new ArmPosition(0.73, -0.23, 45, ArmState.Ready);
    public static final ArmPosition PICK_UP_CONE_MID = new ArmPosition(0.86, 0.71, 45, ArmState.Ready);
    public static final ArmPosition PICK_UP_CONE_HIGH = new ArmPosition(1.17, 1.15, 45, ArmState.Ready);

    public static final ArmPosition PICK_UP_CUBE_LOW = new ArmPosition(0, 0, 0, ArmState.Ready);
    public static final ArmPosition PICK_UP_CUBE_MID = new ArmPosition(0, 0, 0, ArmState.Ready);
    public static final ArmPosition PICK_UP_CUBE_HIGH = new ArmPosition(0, 0, 0, ArmState.Ready);

    public static final ArmPosition SCORE_CONE_LOW = new ArmPosition(0.73, -0.23, 45, ArmState.Ready);
    public static final ArmPosition SCORE_CONE_MID = new ArmPosition(0.86, 0.71, 45, ArmState.Ready);
    public static final ArmPosition SCORE_CONE_HIGH = new ArmPosition(1.17, 1.15, 45, ArmState.Ready);

    public static final ArmPosition SCORE_CUBE_LOW = new ArmPosition(0, 0, 0, ArmState.Ready);
    public static final ArmPosition SCORE_CUBE_MID = new ArmPosition(0, 0, 0, ArmState.Ready);
    public static final ArmPosition SCORE_CUBE_HIGH = new ArmPosition(0, 0, 0, ArmState.Ready);
}
