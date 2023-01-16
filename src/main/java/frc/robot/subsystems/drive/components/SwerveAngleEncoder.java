package frc.robot.subsystems.drive.components;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveAngleEncoder {

    private final CANCoder encoder;

    public SwerveAngleEncoder(int id, double offsetRadians) {

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.magnetOffsetDegrees = Math.toDegrees(offsetRadians);
        config.sensorDirection = false;

        encoder = new CANCoder(id);
        encoder.configAllSettings(config);
        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 100, 250);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }

    public double getRadians() {
        return Math.toRadians(encoder.getAbsolutePosition());
    }
}
