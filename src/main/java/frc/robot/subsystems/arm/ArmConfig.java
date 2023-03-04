package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax.IdleMode;

public class ArmConfig {

    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final double L1_METERS = 0.9525;
    public static final double L2_METERS = 0.81838079;
    public static final double MAX_RADIUS = L1_METERS + L2_METERS;
    public static final double MAX_METERS_PER_SECOND = 0.0762;
    public static final double MIN_X_POS_IN_METERS = 0.85;
    public static final double MIN_Y_POS_IN_METERS = -0.3;

    public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kCoast;

    public static final int ABSOULTE_ENCODER_TICKS_PER_ROTATION = 8192;
    public static final int RELATIVE_ENCODER_TICKS_PER_ROTATION = 42;

    public static final int SHOULDER_RANGE_DEGREES = 90;

    public static final int SHOULDER_MOTOR_ID = 13;
    public static final int SHOULDER_MOTOR_CURRENT_LIMIT = 30;
    public static final boolean SHOULDER_MOTOR_INVERTED = false;
    public static final double SHOULDER_ABSOULTE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / ((5.0 * 4.0 * 3.0) * (70.0 / 24.0));
    public static final double SHOULDER_ABSOLUTE_ENCODER_OFFSET = 72.346;
    public static final boolean SHOULDER_ABSOULTE_ENCODER_INVERTED = false;
    public static final double SHOULDER_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / ((5.0 * 4.0 * 3.0) * (70.0 / 24.0) * (52.0 / 15.0));
    public static final boolean SHOULDER_ANGLE_STARTS_NEGATIVE = false;
    public static final float SHOULDER_FORWARD_LIMIT = 145.0f;
    public static final float SHOULDER_REVERSE_LIMIT = 35.0f;
    public static final double SHOULDER_MAX_SPEED = 0.3;
    public static final PIDParameters SHOULDER_PID = new PIDParameters(0.5, 0.0, 0.0, -SHOULDER_MAX_SPEED, SHOULDER_MAX_SPEED);
    public static final double SHOULDER_SIMULATION_START_ANGLE = 90.0;


    public static final int ELBOW_MOTOR_ID = 12;
    public static final int ELBOW_MOTOR_CURRENT_LIMIT = 30;
    public static final boolean ELBOW_MOTOR_INVERTED = false;
    public static final double ELBOW_ABSOULTE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / (1.0 / 1.0);
    public static final double ELBOW_ABSOULTE_ENCODER_OFFSET = 223.52993392944336;
    public static final boolean ELBOW_ABSOULTE_ENCODER_INVERTED = true;
    public static final double ELBOW_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / ((5.0 * 5.0) * (70.0 / 24.0) * (52.0 / 15.0));
    public static final boolean ELBOW_ANGLE_STARTS_NEGATIVE = true;
    public static final float ELBOW_FORWARD_LIMIT = -40.0f;
    public static final float ELBOW_REVERSE_LIMIT = -320.0f;
    public static final double ELBOW_MAX_SPEED = 0.2;
    public static final PIDParameters ELBOW_PID = new PIDParameters(0.1, 0.0, 0.0, -ELBOW_MAX_SPEED, ELBOW_MAX_SPEED);
    public static final double ELBOW_SIMULATION_START_ANGLE = -165.0;

    public static final int WRIST_MOTOR_ID = 11;
    public static final int WRIST_MOTOR_CURRENT_LIMIT = 20;
    public static final boolean WRIST_MOTOR_INVERTED = false;
    public static final double WRIST_ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR = 360 / ((60.0 / 12.0) * (36.0 / 12.0)) ;
    public static final double WRIST_ABSOLUTE_ENCODER_OFFSET = 0.0;
    public static final boolean WRIST_ABSOULTE_ENCODER_INVERTED = false;
    public static final double WRIST_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR = 360.0 / ((60.0 / 12.0) * (36.0 /12.0) * (36.0 / 18.0));
    public static final boolean WRIST_ANGLE_STARTS_NEGATIVE = false;
    public static final float WRIST_FORWARD_LIMIT = 90.0f;
    public static final float WRIST_REVERSE_LIMIT = -90.0f;
    public static final double WRIST_MAX_SPEED = 0.01;
    public static final PIDParameters WRIST_PID = new PIDParameters(0.1, 0.0, 0.0, -WRIST_MAX_SPEED, WRIST_MAX_SPEED);
    public static final double WRIST_SIMULATION_START_ANGLE = 120.0;
}
