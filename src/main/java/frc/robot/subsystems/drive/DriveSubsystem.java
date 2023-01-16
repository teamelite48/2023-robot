// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.components.SwerveGyro;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class DriveSubsystem extends SubsystemBase{

    private final SwerveGyro gyro = new SwerveGyro(GYRO_ID);

    private final SwerveModule frontLeft = new SwerveModule(
        FRONT_LEFT_DRIVE_MOTOR_ID,
        FRONT_LEFT_TURN_MOTOR_ID,
        FRONT_LEFT_TURN_ENCODER_ID,
        FRONT_LEFT_TURN_OFFSET
    );

    private final SwerveModule frontRight = new SwerveModule(
        FRONT_RIGHT_DRIVE_MOTOR_ID,
        FRONT_RIGHT_TURN_MOTOR_ID,
        FRONT_RIGHT_TURN_ENCODER_ID,
        FRONT_RIGHT_TURN_OFFSET
    );

    private final SwerveModule backLeft = new SwerveModule(
        BACK_LEFT_DRIVE_MOTOR_ID,
        BACK_LEFT_TURN_MOTOR_ID,
        BACK_LEFT_TURN_ENCODER_ID,
        BACK_LEFT_TURN_OFFSET
    );

    private final SwerveModule backRight = new SwerveModule(
        BACK_RIGHT_DRIVE_MOTOR_ID,
        BACK_RIGHT_TURN_MOTOR_ID,
        BACK_RIGHT_TURN_ENCODER_ID,
        BACK_RIGHT_TURN_OFFSET
    );

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
    );

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
    );

    public DriveSubsystem() {
        zeroGyro();
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {

        var swerveModuleStates = getSwerveModuleStates(xSpeed, ySpeed, rotation, fieldRelative);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        odometry.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
    }

    public void zeroGyro() {
        gyro.zero();
    }

    private SwerveModuleState[] getSwerveModuleStates(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {

        var chassisSpeeds = getChasisSpeeds(xSpeed, ySpeed, rotation, fieldRelative);
        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        return swerveModuleStates;
    }

    private ChassisSpeeds getChasisSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
        }
        else {
            return new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
    }
}