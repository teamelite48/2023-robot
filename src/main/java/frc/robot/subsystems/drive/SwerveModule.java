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

    private double targetVelocity = 0;
    private double targetAngle = 0;

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

        double targetAngle = desiredState.angle.getRadians() % (2.0 * Math.PI);
        double targetVelocity = desiredState.speedMetersPerSecond;

        if (targetAngle < 0.0) {
            targetAngle += 2.0 * Math.PI;
        }

        double currentAngle = angleController.getAngle();

        double angleDifference = targetAngle - currentAngle;

        // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
        if (angleDifference >= Math.PI) {
            targetAngle -= 2.0 * Math.PI;
        } else if (angleDifference < -Math.PI) {
            targetAngle += 2.0 * Math.PI;
        }

        angleDifference = targetAngle - currentAngle; // Recalculate difference

        // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
        // movement of the module is less than 90 deg
        if (angleDifference > Math.PI / 2.0 || angleDifference < -Math.PI / 2.0) {
            // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
            targetAngle += Math.PI;
            targetVelocity *= -1.0;
        }

        // Put the target angle back into the range [0, 2pi)
        targetAngle %= (2.0 * Math.PI);

        if (targetAngle < 0.0) {
            targetAngle += 2.0 * Math.PI;
        }

        this.targetVelocity = targetVelocity;
        this.targetAngle = targetAngle;

        driveController.setSpeed(this.targetVelocity);
        angleController.setAngle(this.targetAngle);
    }

    private void initReporting(int driveMotorId) {

        var reportingId = "undefined";

        switch (driveMotorId) {
            case FRONT_LEFT_DRIVE_MOTOR_ID:
                reportingId = "FL";
                break;
            case FRONT_RIGHT_DRIVE_MOTOR_ID:
                reportingId = "FR";
                break;
            case BACK_LEFT_DRIVE_MOTOR_ID:
                reportingId = "BL";
                break;
            case BACK_RIGHT_DRIVE_MOTOR_ID:
                reportingId = "BR";
                break;
        }

        var tab = Shuffleboard.getTab("Swerve Modules");
        var layout = tab.getLayout(reportingId, BuiltInLayouts.kList);

        layout.addDouble("Target Velocity", () -> targetVelocity);
        layout.addDouble("Current Velocity", () -> driveController.getVelocity());

        layout.addDouble("Target Angle", () -> Math.toDegrees(targetAngle));
        layout.addDouble("Current Angle", () -> angleController.getAngle());

        layout.addDouble("Position", () -> driveController.getPosition());

        layout.addDouble("Absolute Angle", () -> angleController.getAbsoluteAngle());
    }
}