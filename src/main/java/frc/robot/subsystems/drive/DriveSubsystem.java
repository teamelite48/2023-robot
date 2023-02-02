// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.drive.DriveConfig.*;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

public class DriveSubsystem extends SubsystemBase{

    enum Gear {
        Low,
        High
    }

    private final Pigeon2 gyro = new Pigeon2(GYRO_ID);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(SLEW_RATE);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SLEW_RATE);
    private Gear gear = Gear.High;

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

    private final SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
        kinematics,
        Rotation2d.fromDegrees(gyro.getYaw()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        }
    );

    public DriveSubsystem() {
        zeroGyro();
        initShuffleBoard();
    }

    public void drive(double x, double y, double rotation, boolean limitSpeed) {

        double speedModifier = 1;

        if(limitSpeed == true) {
            speedModifier = MAX_OUTPUT;

            if(gear == Gear.Low){
                speedModifier = speedModifier * 0.5;
            }
        }

        var desiredStates = getDesiredStates(x * speedModifier, y * speedModifier, rotation * speedModifier);

        setDesiredStates(desiredStates);
    }

    private void setDesiredStates(SwerveModuleState[] desiredStates) {
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setLowGear(){
        gear = Gear.Low;
    }

    public void setHighGear(){
        gear = Gear.High;
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

    // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                // Reset odometry for the first path you run during auto
                if(isFirstPath){
                    odometry.resetPosition(
                        Rotation2d.fromDegrees(gyro.getYaw()),
                        new SwerveModulePosition[] {
                            frontLeft.getPosition(),
                            frontRight.getPosition(),
                            backLeft.getPosition(),
                            backRight.getPosition()
                        },
                        traj.getInitialHolonomicPose()
                    );
                }
            }),

            new PPSwerveControllerCommand(
                traj,
                () -> odometry.getPoseMeters(),
                kinematics, // SwerveDriveKinematics
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this::setDesiredStates, // Module states consumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
            )
        );
    }
}