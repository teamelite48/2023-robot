package frc.robot.subsystems.drive.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveDriveMotor {

    private final CANSparkMax motor;
    private final PIDController pid = new PIDController(1, 0, 0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

    public SwerveDriveMotor(int id) {
        motor = new CANSparkMax(id, MotorType.kBrushless);
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
