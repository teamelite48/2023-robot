// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.components.SwerveDriveEncoder;
import frc.robot.subsystems.drive.components.SwerveDriveMotor;
import frc.robot.subsystems.drive.components.SwerveAngleEncoder;
import frc.robot.subsystems.drive.components.SwerveAngleMotor;

public class SwerveModule {

    private final SwerveDriveMotor driveMotor;
    private final SwerveAngleMotor angleMotor;

    private final SwerveDriveEncoder driveEncoder;
    private final SwerveAngleEncoder angleEncoder;

    private boolean isDriveOpenLoop = true;

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
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optomizedState = SwerveModuleState.optimize(
            desiredState,
            angleEncoder.getRotation2d()
        );

        driveMotor.setSpeed(driveEncoder.getVelocity(), optomizedState.speedMetersPerSecond, isDriveOpenLoop);
        angleMotor.setAngle(angleEncoder.getRadians(), optomizedState.angle.getRadians());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            angleEncoder.getRotation2d()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getDistance(),
            angleEncoder.getRotation2d()
        );
    }

    public void setDriveOpenLoop(boolean isDriveOpenLoop) {
        this.isDriveOpenLoop = isDriveOpenLoop;
    }
}