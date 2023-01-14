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


    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final SwerveModule frontLeft = new SwerveModule(1, 2, 0);
    private final SwerveModule frontRight = new SwerveModule(3, 4, 4);
    private final SwerveModule backLeft = new SwerveModule(5, 6, 8);
    private final SwerveModule backRight = new SwerveModule(7, 8, 12);

    private final SwerveGyro gyro = new SwerveGyro(GYRO_ID);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
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

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        var swerveModuleStates = getSwerveModuleStates(xSpeed, ySpeed, rot, fieldRelative);

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

    private SwerveModuleState[] getSwerveModuleStates(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        var chassisSpeeds = getChasisSpeeds(xSpeed, ySpeed, rot, fieldRelative);
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