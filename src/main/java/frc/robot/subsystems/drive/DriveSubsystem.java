// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    public static final double kMaxSpeed = 3.0; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule frontLeft = new SwerveModule(1, 2, 0, 1, 2);
    private final SwerveModule frontRight = new SwerveModule(3, 4, 4, 5, 6);
    private final SwerveModule backLeft = new SwerveModule(5, 6, 8, 9, 10);
    private final SwerveModule backRight = new SwerveModule(7, 8, 12, 13, 14);

    private final Pigeon2 gyro = new Pigeon2(5);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
    );

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        kinematics,
        getRotation2dFromGyro(),
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

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        
        var chassisSpeeds = getChasisSpeeds(xSpeed, ySpeed, rot, fieldRelative);
        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
            
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
        
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void updateOdometry() {
        odometry.update(
            getRotation2dFromGyro(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }
        );
    }

    public void zeroGyro() {
        gyro.zeroGyroBiasNow();
    }

    private Rotation2d getRotation2dFromGyro() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    private ChassisSpeeds getChasisSpeeds(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (fieldRelative) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2dFromGyro());
        }
        else {
            return new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
    }
}