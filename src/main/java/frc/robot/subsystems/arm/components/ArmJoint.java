package frc.robot.subsystems.arm.components;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmJoint {

    private final CANSparkMax motor;
    private final SparkMaxAbsoluteEncoder absoluteEncoder;
    private final SparkMaxPIDController pidController;

    private double targetAngle = 0;

    public ArmJoint(String jointName, int canId, int absoluteEncoderCanId, double encoderReduction, double offsetDegrees, boolean inverted) {

        motor = new CANSparkMax(canId, MotorType.kBrushless);
        motor.enableVoltageCompensation(12.0); // TODO: combine with nominal voltage in drive config
        motor.setSmartCurrentLimit((int) JOINT_MOTOR_CURRENT_LIMIT);
        motor.setInverted(inverted);
        motor.setIdleMode(MOTOR_IDLE_MODE);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus0, 100);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkMax.PeriodicFrame.kStatus2, 20);

        absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(encoderReduction);
        absoluteEncoder.setZeroOffset(offsetDegrees);

        pidController = motor.getPIDController();
        pidController.setP(1.0);
        pidController.setI(0.0);
        pidController.setD(0.1);
        pidController.setFeedbackDevice(absoluteEncoder);

        initDashboard(jointName);
    }

    public void setAngle(double targetAngle) {
        this.targetAngle = targetAngle;
        pidController.setReference(targetAngle, CANSparkMax.ControlType.kPosition);
    }

    private void initDashboard(String jointName) {

        var tab = Shuffleboard.getTab("Arm");
        var layout = tab.getLayout(jointName, BuiltInLayouts.kList);

        layout.addDouble("Target Angle", () -> this.targetAngle);
        layout.addDouble("Current Angle", () -> this.absoluteEncoder.getPosition());
    }
}
