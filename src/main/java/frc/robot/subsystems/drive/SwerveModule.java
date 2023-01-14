// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class SwerveModule {
    private static final double kWheelRadius = 0.0508;
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final CANCoder turningEncoder;

    private final PIDController drivePID = new PIDController(1, 0, 0);
    private final ProfiledPIDController turningPID = new ProfiledPIDController(
        1,
        0,
        0,
        new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public SwerveModule(
        int driveMotorChannel,
        int turningMotorChannel,
        int driveEncoderChannelA,
        int driveEncoderChannelB,
        int turningEncoderId
    ) {
        driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = new CANCoder(turningEncoderId);

        driveEncoder.setVelocityConversionFactor((2 * Math.PI * kWheelRadius) / 60);
        driveEncoder.setPositionConversionFactor(2* Math.PI * kWheelRadius);

        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),
            getRotation2dFromTurnMotor()
        );
    }


    private Rotation2d getRotation2dFromTurnMotor() {
        return Rotation2d.fromDegrees(turningEncoder.getPosition());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),
            getRotation2dFromTurnMotor()
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {        

        SwerveModuleState optomizedState = SwerveModuleState.optimize(
            desiredState,
            getRotation2dFromTurnMotor()
        );

        setDriveSpeed(optomizedState.speedMetersPerSecond);
        setTurnAngle(optomizedState);
    }

    private void setDriveSpeed(double speedMetersPerSecond) {
    
        double output = drivePID.calculate(driveEncoder.getVelocity(), speedMetersPerSecond);
        double feedforward = driveFeedforward.calculate(speedMetersPerSecond);
        
        driveMotor.setVoltage(output + feedforward);
    }

    private void setTurnAngle(SwerveModuleState state) {

        final double turnOutput = turningPID.calculate(
            Math.toRadians(turningEncoder.getPosition()),
            state.angle.getRadians()
        );

        double turnFeedforwardValue = turnFeedforward.calculate(turningPID.getSetpoint().velocity);

        turningMotor.setVoltage(turnOutput + turnFeedforwardValue);
    }
}