package frc.robot.subsystems.drive.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveTurnMotor {

    private final CANSparkMax motor;

    private final ProfiledPIDController pid = TURN_MOTOR_PID;
    private final SimpleMotorFeedforward feedforward = TURN_MOTOR_FEEDFORWARD;

    public SwerveTurnMotor(int id) {

        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit((int) TURN_CURRENT_LIMIT);

        motor.setInverted(TURN_INVERTED);

        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    private void setVoltage(double outputVolts) {
        motor.setVoltage(outputVolts);
    }

    public void setAngle(double current, double target) {
        double output = pid.calculate(current, target);
        double feedforwardValue = feedforward.calculate(pid.getSetpoint().velocity);

        setVoltage(output + feedforwardValue);
    }
}
