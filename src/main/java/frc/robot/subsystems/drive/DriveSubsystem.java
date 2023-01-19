// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.components.SwerveGyro;
import static frc.robot.subsystems.drive.DriveConfig.*;

public class DriveSubsystem extends SubsystemBase{

    private final SwerveGyro gyro = new SwerveGyro(GYRO_ID);

    private final SlewRateLimiter xSlew = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter ySlew = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter rotationSlew = new SlewRateLimiter(SLEW_RATE);

    private boolean isFieldRelative = false;

    private final SwerveModule frontLeft = new SwerveModule(
        FRONT_LEFT_DRIVE_MOTOR_ID,
        FRONT_LEFT_ANGLE_MOTOR_ID,
        FRONT_LEFT_ANGLE_ENCODER_ID,
        FRONT_LEFT_ANGLE_OFFSET
    );

    private final SwerveModule frontRight = new SwerveModule(
        FRONT_RIGHT_DRIVE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_ENCODER_ID,
        FRONT_RIGHT_ANGLE_OFFSET
    );

    private final SwerveModule backLeft = new SwerveModule(
        BACK_LEFT_DRIVE_MOTOR_ID,
        BACK_LEFT_ANGLE_MOTOR_ID,
        BACK_LEFT_ANGLE_ENCODER_ID,
        BACK_LEFT_ANGLE_OFFSET
    );

    private final SwerveModule backRight = new SwerveModule(
        BACK_RIGHT_DRIVE_MOTOR_ID,
        BACK_RIGHT_ANGLE_MOTOR_ID,
        BACK_RIGHT_ANGLE_ENCODER_ID,
        BACK_RIGHT_ANGLE_OFFSET
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

    public void drive(double xSpeed, double ySpeed, double rotation) {

        var desiredStates = getDesiredStates(
            xSlew.calculate(xSpeed) * MAX_SPEED * MAX_OUTPUT,
            ySlew.calculate(ySpeed) * MAX_SPEED * MAX_OUTPUT,
            rotationSlew.calculate(rotation) * MAX_ANGULAR_SPEED * MAX_OUTPUT
        );

        SmartDashboard.putNumber("Debug", desiredStates[2].speedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
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

    public void setFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative = isFieldRelative;
    }

    public void setDriveOpenLoop(boolean isOpenLoop) {
        frontLeft.setDriveOpenLoop(isOpenLoop);
        frontRight.setDriveOpenLoop(isOpenLoop);
        backLeft.setDriveOpenLoop(isOpenLoop);
        backRight.setDriveOpenLoop(isOpenLoop);
    }

    private SwerveModuleState[] getDesiredStates(double xSpeed, double ySpeed, double rotation) {

        var chassisSpeeds = getChasisSpeeds(xSpeed, ySpeed, rotation);
        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED);

        return swerveModuleStates;
    }

    private ChassisSpeeds getChasisSpeeds(double xSpeed, double ySpeed, double rot) {
        if (isFieldRelative) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
        }
        else {
            return new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
    }
}