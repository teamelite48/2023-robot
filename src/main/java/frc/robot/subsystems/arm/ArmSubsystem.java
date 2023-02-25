// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.components.ArmJoint;
import static frc.robot.subsystems.arm.ArmConfig.*;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    Ready,
    Stowed,
    Tansitioning
  }

  private ArmState armState = ArmState.Stowed;

  private final ArmJoint shoulderJoint = new ArmJoint(
    SHOULDER_MOTOR_ID,
    SHOULDER_ENCODER_REDUCTION,
    SHOULDER_ENCODER_OFFSET,
    SHOULDER_MOTOR_INVERTED,
    SHOULDER_ENCODER_INVERTED,
    SHOULDER_REVERSE_LIMIT_DEGREES,
    SHOULDER_FORWARD_LIMIT_DEGREES
  );

  private final ArmJoint elbowJoint = new ArmJoint(
    ELBOW_MOTOR_ID,
    ELBOW_ENCODER_REDUCTION,
    ELBOW_ENCODER_OFFSET,
    ELBOW_MOTOR_INVERTED,
    ELBOW_ENCODER_INVERTED,
    ELBOW_REVERSE_LIMIT_DEGREES,
    ELBOW_FORWARD_LIMIT_DEGREES
  );

  private final ArmJoint wristJoint = new ArmJoint(
    WRIST_MOTOR_ID,
    WRIST_ENCODER_REDUCTION,
    WRIST_ENCODER_OFFSET,
    WRIST_MOTOR_INVERTED,
    WRIST_ENCODER_INVERTED,
    WRIST_REVERSE_LIMIT_DEGREES,
    WRIST_FORWARD_LIMIT_DEGREES
  );

  public ArmSubsystem() {
    initDashboard();
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      shoulderJoint.simulate();
      elbowJoint.simulate();
      wristJoint.simulate();
    }
  }

  public void increaseWristAngle() {
    wristJoint.setTargetAngle(wristJoint.getCurrentAngle() + 1);
  }

  public void decreaseWristAngle() {
    wristJoint.setTargetAngle(wristJoint.getCurrentAngle() - 1);
  }

  public void setArmState(ArmState armState) {
    this.armState = armState;
  }

  public ArmState getArmState() {
    return this.armState;
  }

  public void setPosition(double x, double y, double wristDegrees) {

    var a = ArmConfig.L1_LENGTH;
    var b = ArmConfig.L2_LENGTH;

    var r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    var alpha = lawOfCosines(y, r, x);
    var beta = lawOfCosines(b, a, r);

    var theta1 = 0.0;

    if (y >= 0) {
      theta1 = alpha + beta;
    }
    else {
      theta1 = -alpha + beta;
    }

    var gamma = lawOfCosines(r, a, b);
    var theta2 = -Math.PI + gamma;

    var theta3 = -theta2 - theta1 + Math.toRadians(wristDegrees);

    shoulderJoint.setTargetAngle(Math.toDegrees(theta1));
    elbowJoint.setTargetAngle(Math.toDegrees(theta2));
    wristJoint.setTargetAngle(Math.toDegrees(theta3));
  }

  public Translation2d getArmPosition() {

    double l1 = ArmConfig.L1_LENGTH;
		double theta1 = Math.toRadians(shoulderJoint.getCurrentAngle());

		Translation2d elbow = new Translation2d(
			l1 * Math.cos(theta1),
			l1 * Math.sin(theta1)
		);

    double l2 = ArmConfig.L2_LENGTH;
		double theta2 = Math.toRadians(elbowJoint.getCurrentAngle());

		Translation2d effector = new Translation2d(
			elbow.getX() + (l2 * Math.cos(theta1 + theta2)),
			elbow.getY() + (l2 * Math.sin(theta1 + theta2))
		);

    return effector;
  }

  private double lawOfCosines(double a, double b, double c) {
    return Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2)) / (2 * b * c));
  }

  private void initDashboard() {

    var tab = Shuffleboard.getTab("Arm");

    var shoulderLayout = tab.getLayout("Shoulder", BuiltInLayouts.kList);
    var elbowLayout = tab.getLayout("Elbow", BuiltInLayouts.kList);
    var wristLayout = tab.getLayout("Wrist", BuiltInLayouts.kList);

    tab.addString("Arm Mode", () -> this.armState.toString());

    shoulderLayout.addDouble("Target Angle", () -> shoulderJoint.getTargetAngle());
    shoulderLayout.addDouble("Current Angle", () -> shoulderJoint.getCurrentAngle());

    elbowLayout.addDouble("Target Angle", () -> elbowJoint.getTargetAngle());
    elbowLayout.addDouble("Current Angle", () -> elbowJoint.getCurrentAngle());

    wristLayout.addDouble("Target Angle", () -> wristJoint.getTargetAngle());
    wristLayout.addDouble("Current Angle", () -> wristJoint.getCurrentAngle());
  }
}