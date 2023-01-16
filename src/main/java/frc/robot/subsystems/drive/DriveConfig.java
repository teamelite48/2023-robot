package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class DriveConfig {

    public static final int GYRO_ID = 5;

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 21;
    public static final int FRONT_LEFT_ANGLE_MOTOR_ID = 22;
    public static final int FRONT_LEFT_ANGLE_ENCODER_ID = 31;
    public static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(38.7 + 180);

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 23;
    public static final int FRONT_RIGHT_ANGLE_MOTOR_ID = 24;
    public static final int FRONT_RIGHT_ANGLE_ENCODER_ID = 33;
    public static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(11.3);

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 25;
    public static final int BACK_LEFT_ANGLE_MOTOR_ID = 26;
    public static final int BACK_LEFT_ANGLE_ENCODER_ID = 35;
    public static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(122.1 + 180);

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 27;
    public static final int BACK_RIGHT_ANGLE_MOTOR_ID = 28;
    public static final int BACK_RIGHT_ANGLE_ENCODER_ID = 37;
    public static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(24.6);

    public static final double MAX_SPEED = 3.0; // 3 meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI;

    public static final double WHEEL_DIAMETER = 0.10033;
    public static final double TRACKWIDTH_METERS = 0.66675;
    public static final double WHEELBASE_METERS = 0.57785;

    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double DRIVE_MOTOR_CURRENT_LIMIT = 80.0;
    public static final double ANGLE_MOTOR_CURRENT_LIMIT = 20.0;

    public static final boolean DRIVE_MOTOR_INVERTED = true;
    public static final boolean ANGLE_MOTOR_INVERTED = false;

    public static final double DRIVE_MOTOR_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double ANGLE_MOTOR_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    public final static double POSITION_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER * DRIVE_MOTOR_REDUCTION;

    public static final PIDController DRIVE_MOTOR_PID = new PIDController(1.0, 0, 0.1);
    public static final SimpleMotorFeedforward DRIVE_MOTOR_FEEDFORWARD = new SimpleMotorFeedforward(0, 0);

    public static final ProfiledPIDController ANGLE_MOTOR_PID = new ProfiledPIDController(1.0, 0, 0.1, new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION));
    public static final SimpleMotorFeedforward ANGLE_MOTOR_FEEDFORWARD = new SimpleMotorFeedforward(0, 0);
}
