// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.drive.DriveConfig.*;

import com.ctre.phoenix.sensors.Pigeon2;

public class DriveSubsystem extends SubsystemBase{

    private final Pigeon2 gyro = new Pigeon2(GYRO_ID);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SLEW_RATE);


    private final SwerveModule frontLeft = new SwerveModule(
        FRONT_LEFT_DRIVE_MOTOR_ID,
        FRONT_LEFT_ANGLE_MOTOR_ID,
        FRONT_LEFT_ANGLE_ENCODER_ID,
        FRONT_LEFT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule frontRight = new SwerveModule(
        FRONT_RIGHT_DRIVE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_MOTOR_ID,
        FRONT_RIGHT_ANGLE_ENCODER_ID,
        FRONT_RIGHT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule backLeft = new SwerveModule(
        BACK_LEFT_DRIVE_MOTOR_ID,
        BACK_LEFT_ANGLE_MOTOR_ID,
        BACK_LEFT_ANGLE_ENCODER_ID,
        BACK_LEFT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveModule backRight = new SwerveModule(
        BACK_RIGHT_DRIVE_MOTOR_ID,
        BACK_RIGHT_ANGLE_MOTOR_ID,
        BACK_RIGHT_ANGLE_ENCODER_ID,
        BACK_RIGHT_ANGLE_OFFSET_DEGREES
    );

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
        new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0)
    );

    public DriveSubsystem() {
        zeroGyro();
        initShuffleBoard();
    }

    public void drive(double x, double y, double rotation) {

        var desiredStates = getDesiredStates(x, y, rotation);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public double getPitch() {
        return gyro.getRoll();
    }

    private SwerveModuleState[] getDesiredStates(double x, double y, double rotation) {

        var chassisSpeeds = getChasisSpeeds(x, y, rotation);
        var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_METERS_PER_SECOND);

        return swerveModuleStates;
    }

    private ChassisSpeeds getChasisSpeeds(double x, double y, double rotation) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            -yLimiter.calculate(y) * MAX_METERS_PER_SECOND,
            -xLimiter.calculate(x) * MAX_METERS_PER_SECOND,
            -rotationLimiter.calculate(rotation) * MAX_ANGULAR_METERS_PER_SECOND,
            Rotation2d.fromDegrees(gyro.getYaw())
        );
    }

    private void initShuffleBoard() {
        var driveTab = Shuffleboard.getTab("Drive");
        driveTab.addDouble("Pitch", () -> getPitch());
    }
}