package frc.robot.subsystems.drive.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveTurnMotor {

    private final CANSparkMax motor;
    private final ProfiledPIDController pid = new ProfiledPIDController(
        1,
        0,
        0,
        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION));

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 0.5);


    public SwerveTurnMotor(int id) {
        motor = new CANSparkMax(id, MotorType.kBrushless);
        pid.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveDriveEncoder getEncoder() {
        return new SwerveDriveEncoder(motor.getEncoder());
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
