package frc.robot.subsystems.arm.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmJoint {

    private final CANSparkMax motorController;
    private final SparkMaxAbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController pidController;

    private double targetAngle;
    private double currentAngle;

    public ArmJoint(
        int canId,
        double encoderReduction,
        double offsetDegrees,
        boolean motorInverted,
        boolean encoderInverted,
        float forwardLimitDegrees,
        float reverseLimitDegrees,
        double simulationAngle
    ) {

        if (Robot.isSimulation()) {
            targetAngle = simulationAngle;
            currentAngle = simulationAngle;
        }

        motorController = new CANSparkMax(canId, MotorType.kBrushless);

        motorController.enableVoltageCompensation(12.0); // TODO: combine with nominal voltage in drive config
        motorController.setSmartCurrentLimit((int) JOINT_MOTOR_CURRENT_LIMIT);

        motorController.setInverted(motorInverted);
        motorController.setIdleMode(MOTOR_IDLE_MODE);

        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motorController.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        motorController.setSoftLimit(SoftLimitDirection.kForward, forwardLimitDegrees);
        motorController.enableSoftLimit(SoftLimitDirection.kForward, true);

        motorController.setSoftLimit(SoftLimitDirection.kReverse, reverseLimitDegrees);
        motorController.enableSoftLimit(SoftLimitDirection.kReverse, true);

        absoluteEncoder = motorController.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(encoderReduction);
        absoluteEncoder.setZeroOffset(offsetDegrees);
        absoluteEncoder.setInverted(encoderInverted);

        pidController = motorController.getPIDController();
        pidController.setP(5.0);
        pidController.setI(0.0);
        pidController.setD(0.1);
        pidController.setFeedbackDevice(absoluteEncoder);
    }

    public void simulate() {
        if (Robot.isSimulation()) {
            double error = this.getTargetAngle() - this.getCurrentAngle();

            if (Math.abs(error) > 0.5) {
                double delta = error / Math.abs(error);
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

    public double getCurrentAngle() {
        if (RobotBase.isSimulation()) {
            return this.currentAngle;
        }
        else {
            return this.absoluteEncoder.getPosition();
        }
    }
}
