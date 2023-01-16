package frc.robot.subsystems.drive.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveDriveMotor {

    private final CANSparkMax motor;

    private final PIDController pid = DRIVE_MOTOR_PID;
    private final SimpleMotorFeedforward feedforward = DRIVE_MOTOR_FEEDFORWARD;

    public SwerveDriveMotor(int id) {

        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.setInverted(DRIVE_INVERTED);

        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit((int) DRIVE_CURRENT_LIMIT);

        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public SwerveDriveEncoder getEncoder() {
        return new SwerveDriveEncoder(motor.getEncoder());
    }

    public void setSpeed(double currentSpeed, double targetSpeed) {

        var output = pid.calculate(currentSpeed, targetSpeed);
        var feedforwardValue = feedforward.calculate(targetSpeed);

        setVoltage(output + feedforwardValue);
    }

    private void setVoltage(double outputVolts) {
        motor.setVoltage(outputVolts);
    }
}
