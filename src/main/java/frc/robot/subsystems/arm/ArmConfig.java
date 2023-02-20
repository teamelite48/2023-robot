package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

public class ArmConfig {

    public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kCoast;

    public static final int SHOULDER_MOTOR_ID = 99;
    public static final boolean SHOULDER_MOTOR_INVERTED = false;
    public static final double SHOULDER_ENCODER_REDUCTION = 0;
    public static final double SHOULDER_ENCODER_OFFSET = 0;

    public static final int ELBOW_MOTOR_ID = 98;
    public static final boolean ELBOW_MOTOR_INVERTED = false;
    public static final double ELBOW_ENCODER_REDUCTION = 0;
    public static final double ELBOW_ENCODER_OFFSET = 0;

    public static final int WRIST_MOTOR_ID = 97;
    public static final boolean WRIST_MOTOR_INVERTED = false;
    public static final double WRIST_ENCODER_REDUCTION = 0;
    public static final double WRIST_ENCODER_OFFSET = 0;

    public static final double JOINT_MOTOR_CURRENT_LIMIT = 80.0;
}
