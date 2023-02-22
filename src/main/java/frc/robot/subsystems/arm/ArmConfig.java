package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

public class ArmConfig {


    public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kCoast;
    public static final int ENCODER_TICKS_PER_ROTATION = 8192;

    public static final int SHOULDER_RANGE_DEGREES = 90;

    public static final int SHOULDER_MOTOR_ID = 99;
    public static final boolean SHOULDER_MOTOR_INVERTED = false;
    public static final double SHOULDER_ENCODER_REDUCTION = 0;
    public static final double SHOULDER_ENCODER_OFFSET = 0;
    public static final boolean SHOULDER_ENCODER_INVERTED = false;
    public static final float SHOULDER_FORWARD_LIMIT_DEGREES = 90;
    public static final float SHOULDER_REVERSE_LIMIT_DEGREES = -90;

    public static final int ELBOW_MOTOR_ID = 98;
    public static final boolean ELBOW_MOTOR_INVERTED = false;
    public static final double ELBOW_ENCODER_REDUCTION = 0;
    public static final double ELBOW_ENCODER_OFFSET = 0;
    public static final boolean ELBOW_ENCODER_INVERTED = false;
    public static final float ELBOW_FORWARD_LIMIT_DEGREES = 90;
    public static final float ELBOW_REVERSE_LIMIT_DEGREES = -90;

    public static final int WRIST_MOTOR_ID = 97;
    public static final boolean WRIST_MOTOR_INVERTED = false;
    public static final double WRIST_ENCODER_REDUCTION = 0;
    public static final double WRIST_ENCODER_OFFSET = 0;
    public static final boolean WRIST_ENCODER_INVERTED = false;
    public static final float WRIST_FORWARD_LIMIT_DEGREES = 90;
    public static final float WRIST_REVERSE_LIMIT_DEGREES = -90;

    public static final double JOINT_MOTOR_CURRENT_LIMIT = 80.0;
}
