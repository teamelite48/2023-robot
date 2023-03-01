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
import frc.robot.subsystems.arm.ArmJointPID;

public class ArmJoint {

    private final CANSparkMax motorController;
    private final SparkMaxAbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController pidController;
    private final RelativeEncoder relativeEncoder;

    private double targetAngle;
    private double currentAngle;

    public ArmJoint(
        int canId,
        int currentLimit,
        double encoderReduction,
        double offsetDegrees,
        boolean motorInverted,
        boolean encoderInverted,
        float forwardLimitDegrees,
        float reverseLimitDegrees,
        ArmJointPID pid,
        double simulationAngle
    ) {

        if (Robot.isSimulation()) {
            targetAngle = simulationAngle;
            currentAngle = simulationAngle;
        }

        motorController = new CANSparkMax(canId, MotorType.kBrushless);

        motorController.enableVoltageCompensation(12.0); // TODO: combine with nominal voltage in drive config
        motorController.setSmartCurrentLimit(currentLimit);

        motorController.setInverted(true);

        motorController.setIdleMode(MOTOR_IDLE_MODE);

        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motorController.setSoftLimit(SoftLimitDirection.kForward, forwardLimitDegrees);
        motorController.enableSoftLimit(SoftLimitDirection.kForward, false);

        motorController.setSoftLimit(SoftLimitDirection.kReverse, reverseLimitDegrees);
        motorController.enableSoftLimit(SoftLimitDirection.kReverse, false);

        absoluteEncoder = motorController.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(360.0);
        absoluteEncoder.setZeroOffset(130.0);

        absoluteEncoder.setInverted(true);

        relativeEncoder = motorController.getEncoder();
        relativeEncoder.setPositionConversionFactor(encoderReduction);
        relativeEncoder.setPosition(absoluteEncoder.getPosition());

        pidController = motorController.getPIDController();
        pidController.setP(0.);
        pidController.setI(0.0);
        pidController.setD(0.0);
        pidController.setFeedbackDevice(relativeEncoder);
        pidController.setOutputRange(pid.minOutput, pid.maxOutput);
    }

    public void simulate() {
        if (Robot.isSimulation()) {
            double error = this.getTargetAngle() - this.getAbsoulteAngle();

            if (Math.abs(error) > 0.01) {
                double delta = error / 10.0;
                this.currentAngle += delta;
            }
        }
    }

    public void setTargetAngle(double targetAngle) {
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
            return this.currentAngle;
        }
        else {
            return this.absoluteEncoder.getPosition();
        }
    }

    public double getRelativeAngle() {
        return this.relativeEncoder.getPosition();
    }
}
