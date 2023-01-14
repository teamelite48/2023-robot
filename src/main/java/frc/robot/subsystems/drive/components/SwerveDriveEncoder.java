package frc.robot.subsystems.drive.components;

import com.revrobotics.RelativeEncoder;

import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveDriveEncoder {

    private final static double METERS_PER_SECOND_CONVERSION = (2 * Math.PI * WHEEL_RADIUS) / 60;
    private final static double DISTANCE_CONVERSION = 2 * Math.PI * WHEEL_RADIUS;

    RelativeEncoder encoder;

    public SwerveDriveEncoder(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    public double getVelocity() {
        return encoder.getVelocity() * METERS_PER_SECOND_CONVERSION;
    }

    public double getDistance() {
        return encoder.getPosition() * DISTANCE_CONVERSION;
    }
}
