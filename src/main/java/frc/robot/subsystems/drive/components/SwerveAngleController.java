package frc.robot.subsystems.drive.components;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveAngleController {

    private final CANSparkMax motor;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder motorEncoder;
    private final CANCoder absoluteEncoder;

    private int resetIteration = 0;

    public SwerveAngleController(int motorId, int absoluteEncoderId, double offsetRadians) {

        absoluteEncoder = new CANCoder(absoluteEncoderId);
        absoluteEncoder.setPositionToAbsolute();
        absoluteEncoder.configMagnetOffset(Math.toDegrees(offsetRadians));
        absoluteEncoder.configSensorDirection(false);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250);

        motor = new CANSparkMax(motorId, MotorType.kBrushless);

        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit((int) ANGLE_MOTOR_CURRENT_LIMIT);

        motor.setInverted(ANGLE_MOTOR_INVERTED);

        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        motorEncoder = motor.getEncoder();
        motorEncoder.setPositionConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR);
        motorEncoder.setVelocityConversionFactor(ANGLE_POSITION_CONVERSION_FACTOR / 60.0);
        motorEncoder.setPosition(absoluteEncoder.getAbsolutePosition());

        pidController = motor.getPIDController();
        pidController.setP(1.0);
        pidController.setI(0.0);
        pidController.setD(0.1);
        pidController.setFeedbackDevice(motorEncoder);
    }


    public void setAngle(double targetAngle) {

        double currentAngle = motorEncoder.getPosition();

        // Reset the NEO's encoder periodically when the module is not rotating.
        // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
        // end up getting a good reading. If we reset periodically this won't matter anymore.
        if (motorEncoder.getVelocity() < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {

            resetIteration++;

            if (resetIteration >= ENCODER_RESET_ITERATIONS) {
                double absoluteAngle = absoluteEncoder.getAbsolutePosition();
                motorEncoder.setPosition(absoluteAngle);
                currentAngle = absoluteAngle;

                resetIteration = 0;
            }
        } else {
            resetIteration = 0;
        }

        double currentAngleMod = currentAngle % (2.0 * Math.PI);

        if (currentAngleMod < 0.0) {
            currentAngleMod += 2.0 * Math.PI;
        }

        // The reference angle has the range [0, 2pi) but the Neo's encoder can go above that
        double adjustedTargetAngle = targetAngle + currentAngle - currentAngleMod;

        if (targetAngle - currentAngleMod > Math.PI) {
            adjustedTargetAngle -= 2.0 * Math.PI;
        }
        else if (targetAngle - currentAngleMod < -Math.PI) {
            adjustedTargetAngle += 2.0 * Math.PI;
        }

        pidController.setReference(adjustedTargetAngle, CANSparkMax.ControlType.kPosition);
    }

    public double getAngle() {

        return motorEncoder.getPosition();

        // if (angle < 0.0) {
        //     angle += 2.0 * Math.PI;
        // }

        // return angle;
    }

    public double getAbsoluteAngle() {
        return absoluteEncoder.getAbsolutePosition();
    }
}
