package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

public class ArmConfig {

    public static final double L1_METERS = 0.9525;
    public static final double L2_METERS = 0.81838079;
    public static final double MAX_RADIUS = L1_METERS + L2_METERS;
    public static final double SHOULDER_METERS_FROM_GROUND =  0.35034722;
    public static final double MAX_METERS_PER_SECOND = 0.0762;

    public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kCoast;
    public static final int ENCODER_TICKS_PER_ROTATION = 8192;

    public static final int SHOULDER_RANGE_DEGREES = 90;

    public static final int SHOULDER_MOTOR_ID = 13;
    public static final int SHOULDER_MOTOR_CURRENT_LIMIT = 30;
    public static final boolean SHOULDER_MOTOR_INVERTED = false;
    public static final double SHOULDER_ENCODER_REDUCTION = 360.0 / (52.0 / 15.0);
    public static final double SHOULDER_ENCODER_OFFSET = 0;
    public static final boolean SHOULDER_ENCODER_INVERTED = false;
    public static final float SHOULDER_FORWARD_LIMIT_DEGREES = 35;
    public static final float SHOULDER_REVERSE_LIMIT_DEGREES = 145;
    public static final ArmJointPID SHOULDER_PID = new ArmJointPID(1.0, 0.0, 0.1, -0.1, 0.1);

    public static final int ELBOW_MOTOR_ID = 12;
    public static final int ELBOW_MOTOR_CURRENT_LIMIT = 30;
    public static final boolean ELBOW_MOTOR_INVERTED = false;
    public static final double ELBOW_ENCODER_REDUCTION = 360.0;
    public static final double ELBOW_ENCODER_OFFSET = 0;
    public static final boolean ELBOW_ENCODER_INVERTED = false;
    public static final float ELBOW_FORWARD_LIMIT_DEGREES = 90;
    public static final float ELBOW_REVERSE_LIMIT_DEGREES = -90;
    public static final ArmJointPID ELBOW_PID = new ArmJointPID(1.0, 0.0, 0.1, -0.1, 0.1);


    public static final int WRIST_MOTOR_ID = 11;
    public static final int WRIST_MOTOR_CURRENT_LIMIT = 20;
    public static final boolean WRIST_MOTOR_INVERTED = false;
    public static final double WRIST_ENCODER_REDUCTION = 360.0 / 2.0;
    public static final double WRIST_ENCODER_OFFSET = 0;
    public static final boolean WRIST_ENCODER_INVERTED = false;
    public static final float WRIST_FORWARD_LIMIT_DEGREES = 90;
    public static final float WRIST_REVERSE_LIMIT_DEGREES = -90;
    public static final ArmJointPID WRIST_PID = new ArmJointPID(1.0, 0.0, 0.1, -0.1, 0.1);
}
