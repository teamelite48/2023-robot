// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.components.ArmJoint;
import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmSubsystem extends SubsystemBase {

  private final ArmJoint shoulderJoint = new ArmJoint(
    "Shoulder",
    SHOULDER_MOTOR_ID,
    SHOULDER_ENCODER_REDUCTION,
    SHOULDER_ENCODER_OFFSET,
    SHOULDER_MOTOR_INVERTED,
    SHOULDER_ENCODER_INVERTED,
    SHOULDER_REVERSE_LIMIT_DEGREES,
    SHOULDER_FORWARD_LIMIT_DEGREES
  );

  private final ArmJoint elbowJoint = new ArmJoint(
    "Elbow",
    ELBOW_MOTOR_ID,
    ELBOW_ENCODER_REDUCTION,
    ELBOW_ENCODER_OFFSET,
    ELBOW_MOTOR_INVERTED,
    ELBOW_ENCODER_INVERTED,
    ELBOW_REVERSE_LIMIT_DEGREES,
    ELBOW_FORWARD_LIMIT_DEGREES
  );

  private final ArmJoint wristJoint = new ArmJoint(
    "Wrist",
    WRIST_MOTOR_ID,
    WRIST_ENCODER_REDUCTION,
    WRIST_ENCODER_OFFSET,
    WRIST_MOTOR_INVERTED,
    WRIST_ENCODER_INVERTED,
    WRIST_REVERSE_LIMIT_DEGREES,
    WRIST_FORWARD_LIMIT_DEGREES
  );

  public ArmSubsystem() {
  }

  @Override
  public void periodic() {
  }

  public void increaseWristAngle() {
    wristJoint.setAngle(wristJoint.getAngle() + 1);
  }

  public void decreaseWristAngle() {
    wristJoint.setAngle(wristJoint.getAngle() - 1);
  }
}
