// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.drive.components.SwerveDriveController;
import frc.robot.subsystems.drive.components.SwerveAngleController;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveModule {

    private final SwerveDriveController driveController;
    private final SwerveAngleController angleController;

    public SwerveModule(
        int driveMotorId,
        int angleMotorId,
        int absoluteEncoderId,
        double angleOffset
    ) {
        driveController = new SwerveDriveController(driveMotorId);
        angleController = new SwerveAngleController(angleMotorId, absoluteEncoderId, angleOffset);

        initReporting(driveMotorId);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        double desiredAngle = desiredState.angle.getRadians() % (2.0 * Math.PI);
        double desiredVelocity = desiredState.speedMetersPerSecond;

        if (desiredAngle < 0.0) {
            desiredAngle += 2.0 * Math.PI;
        }

        double currentAngle = angleController.getCurrentAngle();

        double angleDifference = desiredAngle - currentAngle;

        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (angleDifference >= Math.PI) {
            desiredAngle -= 2.0 * Math.PI;
        } else if (angleDifference < -Math.PI) {
            desiredAngle += 2.0 * Math.PI;
        }

        angleDifference = desiredAngle - currentAngle; // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (angleDifference > Math.PI / 2.0 || angleDifference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            desiredAngle += Math.PI;
            desiredVelocity *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        desiredAngle %= (2.0 * Math.PI);

        if (desiredAngle < 0.0) {
            desiredAngle += 2.0 * Math.PI;
        }

        driveController.setSpeed(desiredVelocity);
        angleController.setAngle(desiredAngle);
    }

    private void initReporting(int driveMotorId) {

        var reportingId = "undefined";

        switch (driveMotorId) {
            case FRONT_LEFT_DRIVE_MOTOR_ID:
                reportingId = "Front Left";
                break;
            case FRONT_RIGHT_DRIVE_MOTOR_ID:
                reportingId = "Front Right";
                break;
            case BACK_LEFT_DRIVE_MOTOR_ID:
                reportingId = "Back Left";
                break;
            case BACK_RIGHT_DRIVE_MOTOR_ID:
                reportingId = "Back Right";
                break;
        }

        var tab = Shuffleboard.getTab("Swerve Modules");
        var layout = tab.getLayout(reportingId, BuiltInLayouts.kList);

        layout.addDouble("Target Velocity", () -> driveController.getTargetVelocity());
        layout.addDouble("Current Velocity", () -> driveController.getCurrentVelocity());

        layout.addDouble("Target Angle", () -> Math.toDegrees(angleController.getTargetAngle()));
        layout.addDouble("Current Angle", () -> Math.toDegrees(angleController.getCurrentAngle()));

        layout.addDouble("Absolute Angle", () -> Math.toDegrees(angleController.getAbsoluteAngle()));

        layout.addDouble("Position", () -> driveController.getCurrentPosition());
    }
}