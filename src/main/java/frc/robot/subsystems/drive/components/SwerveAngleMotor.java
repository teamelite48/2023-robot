package frc.robot.subsystems.drive.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveAngleMotor {

    private final CANSparkMax motor;
    private final SparkMaxPIDController pidController;
    private final ProfiledPIDController pid = ANGLE_MOTOR_PID;

    public SwerveAngleMotor(int id, SwerveAngleEncoder angleEncoder) {

        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit((int) ANGLE_MOTOR_CURRENT_LIMIT);

        motor.setInverted(ANGLE_MOTOR_INVERTED);

        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        pid.enableContinuousInput(-Math.PI, Math.PI);

        var integratedEncoder = motor.getEncoder();

        integratedEncoder.setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        integratedEncoder.setVelocityConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR / 60.0);
        integratedEncoder.setPosition(angleEncoder.getRadians());

        pidController = motor.getPIDController();
        pidController.setP(pid.getP());
        pidController.setI(pid.getI());
        pidController.setD(pid.getD());
        pidController.setIZone(0);
        pidController.setFF(0);

        pidController.setOutputRange(-MAX_OUTPUT, MAX_OUTPUT);

        pidController.setFeedbackDevice(integratedEncoder);
    }

    public void setAngle(double current, double target) {
        pidController.setReference(target, CANSparkMax.ControlType.kPosition);
    }

    // public void setAngle(double current, double target) {
    //     double output = pid.calculate(current, target);
    //     double feedforwardValue = feedforward.calculate(pid.getSetpoint().velocity);

    //      setVoltage(output + feedforwardValue);
    // }

    // private void setVoltage(double outputVolts) {
    //     motor.setVoltage(outputVolts);
    // }
}
