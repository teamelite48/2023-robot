// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.subsystems.gripper.GripperConfig.*;

public class GripperSubsystem extends SubsystemBase {

  public enum GripperMode {
    Cone,
    Cube
  }

  private final CANSparkMax motorController;

  private GripperMode gripperMode = GripperMode.Cone;

  public GripperSubsystem() {
    motorController = new CANSparkMax(GRIPPER_MOTOR_ID, MotorType.kBrushless);
    motorController.setSmartCurrentLimit(GRIPPER_MOTOR_CURRENT_LIMIT);

    initDashboard();
  }

  @Override
  public void periodic() {
  }

  public void intake() {
    if (gripperMode == GripperMode.Cone) {
      motorController.set(GRIPPER_MOTOR_SPEED);
    }
    else if (gripperMode == GripperMode.Cube) {
      motorController.set(-GRIPPER_MOTOR_SPEED);
    }
  }

  public void outtake() {
    if (gripperMode == GripperMode.Cone) {
      motorController.set(-GRIPPER_MOTOR_SPEED);
    }
    else if (gripperMode == GripperMode.Cube) {
      motorController.set(GRIPPER_MOTOR_SPEED);
    }
  }

  public void setMode(GripperMode _gripperMode) {
      gripperMode = _gripperMode;
  }

  public GripperMode getMode() {
    return gripperMode;
  }

  public void stop() {
    motorController.set(0);
  }

    private void initDashboard() {

    var tab = Shuffleboard.getTab("Gripper");

    tab.addString("Mode", () -> getMode().toString());
    tab.addDouble("Speed", () -> motorController.get());
    tab.addDouble("Current", () -> motorController.getOutputCurrent());
  }
}
