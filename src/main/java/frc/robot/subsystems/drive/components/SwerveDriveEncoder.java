package frc.robot.subsystems.drive.components;

import com.revrobotics.RelativeEncoder;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveDriveEncoder {

    RelativeEncoder encoder;

    public SwerveDriveEncoder(RelativeEncoder encoder) {

        this.encoder = encoder;

        this.encoder.setPositionConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR);
        this.encoder.setVelocityConversionFactor(DRIVE_POSITION_CONVERSION_FACTOR / 60.0);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getPosition() {
        return encoder.getPosition();
    }
}
