package frc.robot.subsystems.drive.components;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveGyro {

    private final Pigeon2 gyro;

    public SwerveGyro(int id) {
        gyro = new Pigeon2(id);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void zero() {
        gyro.zeroGyroBiasNow();
    }
}
