// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.components.SwerveDriveEncoder;
import frc.robot.subsystems.drive.components.SwerveDriveMotor;
import frc.robot.subsystems.drive.components.SwerveTurnEncoder;
import frc.robot.subsystems.drive.components.SwerveTurnMotor;

public class SwerveModule {

    private final SwerveDriveMotor driveMotor;
    private final SwerveTurnMotor turnMotor;

    private final SwerveDriveEncoder driveEncoder;
    private final SwerveTurnEncoder turnEncoder;


    public SwerveModule(
        int driveMotorId,
        int turnMotorId,
        int turnEncoderId
    ) {
        driveMotor = new SwerveDriveMotor(driveMotorId);
        turnMotor = new SwerveTurnMotor(turnMotorId);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new SwerveTurnEncoder(turnEncoderId);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            turnEncoder.getRotation2d()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getDistance(),
            turnEncoder.getRotation2d()
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optomizedState = SwerveModuleState.optimize(
            desiredState,
            turnEncoder.getRotation2d()
        );

        driveMotor.setSpeed(driveEncoder.getVelocity(), optomizedState.speedMetersPerSecond);
        turnMotor.setAngle(turnEncoder.getRadians(), optomizedState.angle.getRadians());
    }
}