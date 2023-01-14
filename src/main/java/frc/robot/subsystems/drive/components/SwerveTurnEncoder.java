package frc.robot.subsystems.drive.components;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveTurnEncoder {

    private final CANCoder encoder;

    public SwerveTurnEncoder(int id) {
        encoder = new CANCoder(id);
        encoder.setPositionToAbsolute();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

    public double getRadians() {
        return Math.toRadians(encoder.getPosition());
    }
}
