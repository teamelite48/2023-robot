// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.drive.components.SwerveDriveEncoder;
import frc.robot.subsystems.drive.components.SwerveDriveMotor;
import frc.robot.subsystems.drive.components.SwerveAngleEncoder;
import frc.robot.subsystems.drive.components.SwerveAngleMotor;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class SwerveModule {

    private final SwerveDriveMotor driveMotor;
    private final SwerveAngleMotor angleMotor;

    private final SwerveDriveEncoder driveEncoder;
    private final SwerveAngleEncoder angleEncoder;

    private boolean isDriveOpenLoop = true;
    private SwerveModuleState optomizedState = new SwerveModuleState();

    public SwerveModule(
        int driveMotorId,
        int turnMotorId,
        int turnEncoderId,
        double angleOffsetRadians
    ) {
        driveMotor = new SwerveDriveMotor(driveMotorId);
        angleMotor = new SwerveAngleMotor(turnMotorId);

        driveEncoder = driveMotor.getEncoder();
        angleEncoder = new SwerveAngleEncoder(turnEncoderId, angleOffsetRadians);

        initReporting(driveMotorId);
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        optomizedState = SwerveModuleState.optimize(
            desiredState,
            angleEncoder.getRotation()
        );

        driveMotor.setSpeed(driveEncoder.getVelocity(), optomizedState.speedMetersPerSecond, isDriveOpenLoop);
        angleMotor.setAngle(angleEncoder.getRadians(), optomizedState.angle.getRadians());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            angleEncoder.getRotation()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            angleEncoder.getRotation()
        );
    }

    public void setDriveOpenLoop(boolean isDriveOpenLoop) {
        this.isDriveOpenLoop = isDriveOpenLoop;
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

        tab.addDouble(reportingId + " TV", () -> optomizedState.speedMetersPerSecond);
        tab.addDouble(reportingId + " CV", () -> driveEncoder.getVelocity());

        tab.addDouble(reportingId + " TA", () -> optomizedState.angle.getDegrees());
        tab.addDouble(reportingId + " CA", () -> Math.toDegrees(angleEncoder.getRadians()));

        tab.addDouble(reportingId + " Pos", () -> driveEncoder.getPosition());
    }
}