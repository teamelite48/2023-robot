package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

public class ArmConfig {

    public static final double NOMINAL_VOLTAGE = 12.0;

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
    public static final double SHOULDER_ABSOULTE_ENCODER_POSITION_CONVERSION_FACTOR = 0.0;
    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = 0;
    public static final boolean SHOULDER_ABSOULTE_ENCODER_INVERTED = false;
    public static final double SHOULDER_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR = 0.0;
    public static final float SHOULDER_FORWARD_LIMIT = 35.0f;
    public static final float SHOULDER_REVERSE_LIMIT = 145.0f;
    public static final PIDParameters SHOULDER_PID = new PIDParameters(1.0, 0.0, 0.1, -0.1, 0.1);
    public static final double SHOULDER_SIMULATION_START_ANGLE = 90.0;
    public static final double SHOULDER_TEST_SPEED = 0.05;


    public static final int ELBOW_MOTOR_ID = 12;
    public static final int ELBOW_MOTOR_CURRENT_LIMIT = 30;
    public static final boolean ELBOW_MOTOR_INVERTED = true;
    public static final double ELBOW_ABSOULTE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0;
    public static final double ELBOW_ABSOULTE_ENCODER_OFFSET = 130.0;
    public static final boolean ELBOW_ABSOULTE_ENCODER_INVERTED = false;
    public static final double ELBOW_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / 5.0; // 42.0 / 5.0 ???
    public static final float ELBOW_FORWARD_LIMIT = 90.0f;
    public static final float ELBOW_REVERSE_LIMIT = -90.0f;
    public static final PIDParameters ELBOW_PID = new PIDParameters(0.1, 0.0, 0.0, -0.1, 0.1);
    public static final double ELBOW_SIMULATION_START_ANGLE = -165.0;
    public static final double ELBOW_TEST_SPEED = 0.05;

    public static final int WRIST_MOTOR_ID = 11;
    public static final int WRIST_MOTOR_CURRENT_LIMIT = 20;
    public static final boolean WRIST_MOTOR_INVERTED = false;
    public static final double WRIST_ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR = 0.0;
    public static final double WRIST_ABSOLUTE_ENCODER_OFFSET = 0;
    public static final boolean WRIST_ABSOULTE_ENCODER_INVERTED = false;
    public static final double WRIST_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR = 0.0;
    public static final float WRIST_FORWARD_LIMIT = 90.0f;
    public static final float WRIST_REVERSE_LIMIT = -90.0f;
    public static final PIDParameters WRIST_PID = new PIDParameters(1.0, 0.0, 0.1, -0.1, 0.1);
    public static final double WRIST_SIMULATION_START_ANGLE = 120.0;
    public static final double WRIST_TEST_SPEED = 0.05;
}
