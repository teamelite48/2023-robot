package frc.robot.subsystems.drive.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveAngleMotor {

    private final CANSparkMax motor;
    private final ProfiledPIDController pid = ANGLE_MOTOR_PID;
    private final SimpleMotorFeedforward feedforward = ANGLE_MOTOR_FEEDFORWARD;

    public SwerveAngleMotor(int id) {

        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit((int) ANGLE_MOTOR_CURRENT_LIMIT);

        motor.setInverted(ANGLE_MOTOR_INVERTED);

        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void setAngle(double currentRadians, double targetRadians) {
        double outputVolatage = pid.calculate(currentRadians, targetRadians) / Math.PI * NOMINAL_VOLTAGE;
        double feedforwardVoltage = feedforward.calculate(pid.getSetpoint().velocity);

        motor.setVoltage(outputVolatage + feedforwardVoltage);
    }
}
