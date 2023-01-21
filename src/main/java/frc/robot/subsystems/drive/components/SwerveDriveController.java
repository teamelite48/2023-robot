package frc.robot.subsystems.drive.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveDriveController {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public SwerveDriveController(int id) {

        motor = new CANSparkMax(id, MotorType.kBrushless);

        motor.setInverted(DRIVE_MOTOR_INVERTED);

        motor.enableVoltageCompensation(NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit((int) DRIVE_MOTOR_CURRENT_LIMIT);

        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        encoder = motor.getEncoder();

        this.encoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR);
        this.encoder.setVelocityConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR / 60.0);
    }

    public void setSpeed(double metersPerSecond) {
        motor.setVoltage(metersPerSecond / MAX_METERS_PER_SECOND * NOMINAL_VOLTAGE);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}


