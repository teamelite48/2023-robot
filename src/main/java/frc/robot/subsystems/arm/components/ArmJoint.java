package frc.robot.subsystems.arm.components;

import static frc.robot.subsystems.arm.ArmConfig.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.PIDParameters;

public class ArmJoint {

    private final CANSparkMax motorController;
    private final SparkMaxAbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder relativeEncoder;

    private boolean areEncodersInitialized = false;
    private double targetAngle;
    private double currentAngle;

    public ArmJoint(
        int motorId,
        int motorCurrentLimit,
        boolean isMotorInverted,
        double absoluteEncoderPositionConversionFactor,
        double absoluteEncoderOffset,
        boolean isAbsoluteEncoderInverted,
        double relativeEncoderPositionConversionFactor,
        float forwardLimit,
        float reverseLimit,
        PIDParameters pidParams,
        double simulationStartAngle
    ) {

        if (Robot.isSimulation()) {
            areEncodersInitialized = true;
            targetAngle = simulationStartAngle;
            currentAngle = simulationStartAngle;
        }

        motorController = new CANSparkMax(motorId, MotorType.kBrushless);

        motorController.enableVoltageCompensation(NOMINAL_VOLTAGE); // TODO: combine with nominal voltage in drive config
        motorController.setSmartCurrentLimit(motorCurrentLimit);

        motorController.setInverted(isMotorInverted);

        motorController.setIdleMode(MOTOR_IDLE_MODE);

        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motorController.setSoftLimit(SoftLimitDirection.kForward, forwardLimit);
        motorController.enableSoftLimit(SoftLimitDirection.kForward, true);

        motorController.setSoftLimit(SoftLimitDirection.kReverse, reverseLimit);
        motorController.enableSoftLimit(SoftLimitDirection.kReverse, true);

        absoluteEncoder = motorController.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(absoluteEncoderPositionConversionFactor);

        absoluteEncoder.setZeroOffset(absoluteEncoderOffset);

        absoluteEncoder.setInverted(isAbsoluteEncoderInverted);

        relativeEncoder = motorController.getEncoder();
        relativeEncoder.setPositionConversionFactor(relativeEncoderPositionConversionFactor);

        seedRelativeEncoderWithAbsoluteAngle();

        pidController = motorController.getPIDController();
        pidController.setP(pidParams.P);
        pidController.setI(pidParams.I);
        pidController.setD(pidParams.D);
        pidController.setFeedbackDevice(relativeEncoder);
        pidController.setOutputRange(pidParams.MIN_OUTPUT, pidParams.MAX_OUTPUT);
    }

    public void periodic() {
        if (areEncodersInitialized == false) {
            seedRelativeEncoderWithAbsoluteAngle();
        }
    }

    public void seedRelativeEncoderWithAbsoluteAngle() {
        relativeEncoder.setPosition(absoluteEncoder.getPosition());

        if ((getAbsoulteAngle() - getRelativeAngle()) < 0.001) {
            areEncodersInitialized = true;
        }
    }

    public void simulate() {
        if (Robot.isSimulation()) {
            double error = this.getTargetAngle() - this.getRelativeAngle();

            if (Math.abs(error) > 0.01) {
                double delta = error / 10.0;
                this.currentAngle += delta;
            }
        }
    }

    public void setTargetAngle(double targetAngle) {

        if (areEncodersInitialized == false) {
            return;
        }

        this.targetAngle = targetAngle;
        this.pidController.setReference(this.targetAngle, CANSparkMax.ControlType.kPosition);
    }

    public double getTargetAngle() {
        return this.targetAngle;
    }

    public void setMotorSpeed(double speed) {
        motorController.set(speed);
    }

    public double getAbsoulteAngle() {
        if (RobotBase.isSimulation()) {
            return Math.abs(this.currentAngle);
        }
        else {
            return this.absoluteEncoder.getPosition();
        }
    }

    public double getRelativeAngle() {
        if (RobotBase.isSimulation()) {
            return this.currentAngle;
        }
        else {
            return this.relativeEncoder.getPosition();
        }
    }

    public boolean areEncodersInitilized() {
        return areEncodersInitialized;
    }
}
