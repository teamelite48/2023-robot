package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.ArmSubsystem.ArmState;

public class ArmPreset {

    public static final double FRAME_X_BOUNDARY = 0.6;

    public static final ArmPosition STOWED = new ArmPosition(0.28, 0.1, 0.0, ArmState.InsideFramePerimeter);

    public static final ArmPosition LOW_DROP_ZONE = new ArmPosition(0.8, 0.1, 45.0, ArmState.OutsideFramePerimeter);
    public static final ArmPosition HIGH_DROP_ZONE = new ArmPosition(0.8, 0.45, 45.0, ArmState.OutsideFramePerimeter);

    public static final ArmPosition PICK_UP_CONE_LOW = new ArmPosition(0.8, 0.06, -135.0, ArmState.OutsideFramePerimeter);
    public static final ArmPosition PICK_UP_CONE_MID = new ArmPosition(0.39, 0.14, -20.0, ArmState.InsideFramePerimeter);
    public static final ArmPosition PICK_UP_CONE_HIGH = new ArmPosition(0.79, 0.85, -135.0, ArmState.OutsideFramePerimeter);

    public static final ArmPosition PICK_UP_CUBE_LOW= new ArmPosition(0.8, -0.15, 45.0, ArmState.OutsideFramePerimeter);
    public static final ArmPosition PICK_UP_CUBE_MID = new ArmPosition(0.59, 0.28, 160.0, ArmState.InsideFramePerimeter);
    public static final ArmPosition PICK_UP_CUBE_HIGH = new ArmPosition(0.8, 0.75, 45.0, ArmState.OutsideFramePerimeter);

    public static final ArmPosition SCORE_CONE_LOW = new ArmPosition(0.8, 0.07, -135.0, ArmState.OutsideFramePerimeter);
    public static final ArmPosition SCORE_CONE_MID = new ArmPosition(0.89, 0.68, -135.0, ArmState.OutsideFramePerimeter);
    public static final ArmPosition SCORE_CONE_HIGH = new ArmPosition(1.32, 0.9, -135.0, ArmState.OutsideFramePerimeter);

    public static final ArmPosition SCORE_CUBE_LOW= new ArmPosition(0.8, -0.15, 45.0, ArmState.OutsideFramePerimeter);
    public static final ArmPosition SCORE_CUBE_MID = new ArmPosition(0.8, 0.45, 90.0, ArmState.OutsideFramePerimeter);
    public static final ArmPosition SCORE_CUBE_HIGH = new ArmPosition(1.12, 0.65, 90.0, ArmState.OutsideFramePerimeter);
}
