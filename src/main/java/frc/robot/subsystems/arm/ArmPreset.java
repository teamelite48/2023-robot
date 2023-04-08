package frc.robot.subsystems.arm;

public class ArmPreset {

    public static final double FRAME_X_BOUNDARY = 0.6;

    public static final ArmPosition STOWED_CONE = new ArmPosition(0.28, 0.1, -90.0);
    public static final ArmPosition STOWED_CUBE = new ArmPosition(0.28, 0.1, 0.0);

    public static final ArmPosition LOW_DROP_ZONE = new ArmPosition(0.676, 0.1, -90.0);
    public static final ArmPosition HIGH_DROP_ZONE = new ArmPosition(0.4, 0.49, -90.0);

    public static final ArmPosition PICK_UP_CONE_LOW = new ArmPosition(0.676, -0.048, -120.0);
    public static final ArmPosition PICK_UP_CONE_MID = new ArmPosition(0.343, 0.14, -20.0);
    public static final ArmPosition PICK_UP_CONE_HIGH = new ArmPosition(0.79, 0.7708, -130.0);

    public static final ArmPosition PICK_UP_CUBE_LOW= new ArmPosition(0.676, -0.122, -125.0);
    public static final ArmPosition PICK_UP_CUBE_MID = new ArmPosition(0.39, 0.25, -20.0);
    public static final ArmPosition PICK_UP_CUBE_HIGH = new ArmPosition(0.8, 0.75, -125.0);

    public static final ArmPosition SCORE_CONE_LOW = new ArmPosition(0.28, 0.1, -90.0);
    public static final ArmPosition SCORE_CONE_MID = new ArmPosition(0.894, 0.564, -135.0);
    public static final ArmPosition SCORE_CONE_HIGH = new ArmPosition(1.40, 0.791, -135.0);

    public static final ArmPosition SCORE_CUBE_LOW= new ArmPosition(0.28, 0.1, -90.0);
    public static final ArmPosition SCORE_CUBE_MID = new ArmPosition(0.8, 0.501, -90.0);
    public static final ArmPosition SCORE_CUBE_HIGH = new ArmPosition(1.12, 0.65, -90.0);
}