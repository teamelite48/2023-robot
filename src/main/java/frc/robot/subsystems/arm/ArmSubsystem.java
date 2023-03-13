// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.components.ArmJoint;

import static frc.robot.subsystems.arm.ArmConfig.*;

import java.text.DecimalFormat;

public class ArmSubsystem extends SubsystemBase {

  public enum ArmState {
    Ready,
    Transitioning
  }

  public enum ArmOrientation {
    Front,
    Rear
  }

  private ArmState armState = ArmState.Ready;
  private ArmOrientation armOrientation = ArmOrientation.Front;

  private Translation2d position;
  private double wristDegrees = 0.0;

  private final ArmJoint shoulderJoint = new ArmJoint(
    SHOULDER_MOTOR_ID,
    SHOULDER_MOTOR_CURRENT_LIMIT,
    SHOULDER_MOTOR_INVERTED,
    SHOULDER_ABSOULTE_ENCODER_POSITION_CONVERSION_FACTOR,
    SHOULDER_ABSOLUTE_ENCODER_OFFSET,
    SHOULDER_ABSOULTE_ENCODER_INVERTED,
    SHOULDER_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR,
    SHOULDER_FORWARD_LIMIT,
    SHOULDER_REVERSE_LIMIT,
    SHOULDER_PID,
    SHOULDER_START_ANGLE
  );

  private final ArmJoint elbowJoint = new ArmJoint(
    ELBOW_MOTOR_ID,
    ELBOW_MOTOR_CURRENT_LIMIT,
    ELBOW_MOTOR_INVERTED,
    ELBOW_ABSOULTE_ENCODER_POSITION_CONVERSION_FACTOR,
    ELBOW_ABSOULTE_ENCODER_OFFSET,
    ELBOW_ABSOULTE_ENCODER_INVERTED,
    ELBOW_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR,
    ELBOW_FORWARD_LIMIT,
    ELBOW_REVERSE_LIMIT,
    ELBOW_PID,
    ELBOW_START_ANGLE
  );

  private final ArmJoint wristJoint = new ArmJoint(
    WRIST_MOTOR_ID,
    WRIST_MOTOR_CURRENT_LIMIT,
    WRIST_MOTOR_INVERTED,
    WRIST_ABSOLUTE_ENCODER_POSITION_CONVERSION_FACTOR,
    WRIST_ABSOLUTE_ENCODER_OFFSET,
    WRIST_ABSOULTE_ENCODER_INVERTED,
    WRIST_RELATIVE_ENCODER_POSITION_CONVERSION_FACTOR,
    WRIST_FORWARD_LIMIT,
    WRIST_REVERSE_LIMIT,
    WRIST_PID,
    WRIST_START_ANGLE
  );

  public ArmSubsystem() {
    updatePosition();
    initDashboard();
  }

  @Override
  public void periodic() {

    updatePosition();

    shoulderJoint.periodic();
    elbowJoint.periodic();
    wristJoint.periodic();

    if (Robot.isSimulation()) {
      shoulderJoint.simulate();
      elbowJoint.simulate();
      wristJoint.simulate();
    }
  }

  public void setShoulderMotorSpeed(double speed) {
    shoulderJoint.setMotorSpeed(speed);
  }

  public void setElbowMotorSpeed(double speed) {
    elbowJoint.setMotorSpeed(speed);
  }

  public void setWristMotorSpeed(double speed) {
    wristJoint.setMotorSpeed(speed);
  }

  public void setArmState(ArmState armState) {
    this.armState = armState;
  }

  public ArmState getArmState() {
    return this.armState;
  }

  public void manualPosition(double leftYAxis, double rightYAxis) {

    if (this.armState != ArmState.Ready) return;
    if ((Math.abs(leftYAxis) > 0.0 || Math.abs(rightYAxis) > 0.0) == false) return;

    var desiredXPos = this.position.getX() + (leftYAxis * ArmConfig.MAX_MANUAL_SPEED);
    var desiredYPos = this.position.getY() + (rightYAxis * ArmConfig.MAX_MANUAL_SPEED);

    if (isPositionAllowed(desiredXPos, desiredYPos)) {
      setPosition(desiredXPos, desiredYPos, this.wristDegrees);
    }
  }

  public boolean isPositionAllowed(double x, double y) {

    if (x < ArmConfig.MIN_X_POS_IN_METERS || y < ArmConfig.MIN_Y_POS_IN_METERS) {
      return false;
    }

    var yPosUpperBound = Math.sqrt(Math.pow(ArmConfig.MAX_RADIUS, 2) - Math.pow(x, 2));

    if (y > yPosUpperBound) {
      return false;
    }

    var xPosUpperBound = Math.sqrt(Math.pow(ArmConfig.MAX_RADIUS, 2) - Math.pow(y, 2));

    if (x > xPosUpperBound) {
      return false;
    }

    return true;
  }

  public void setPosition(double x, double y, double wristDegrees) {

    this.wristDegrees = wristDegrees;

    double a = ArmConfig.L1_METERS;
    double b = ArmConfig.L2_METERS;

    double r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    double alpha = lawOfCosines(y, r, x);
    double beta = lawOfCosines(b, a, r);

    double theta1 = 0.0;

    if (y >= 0) {
      theta1 = alpha + beta;
    }
    else {
      theta1 = -alpha + beta;
    }

    double gamma = lawOfCosines(r, a, b);
    double theta2 = -Math.PI + gamma;

    double theta3 = -theta2 - theta1 + Math.toRadians(this.wristDegrees);

    double theta2WithOffset = applyOffsetToTheta2(theta1, theta2);

    if (this.armOrientation == ArmOrientation.Rear) {
      theta1 = Math.toRadians(180.0) - theta1;
      theta2WithOffset = Math.toRadians(-360.0) - theta2WithOffset;
      theta3 *= -1.0;
    }

    shoulderJoint.setTargetAngle(Math.toDegrees(theta1));
    elbowJoint.setTargetAngle(Math.toDegrees(theta2WithOffset));
    wristJoint.setTargetAngle(Math.toDegrees(theta3));
  }

  public void updatePosition() {

    double l1 = ArmConfig.L1_METERS;
		double theta1 = Math.toRadians(shoulderJoint.getRelativeAngle());

		Translation2d elbowPosition = new Translation2d(
			l1 * Math.cos(theta1),
			l1 * Math.sin(theta1)
		);

    double l2 = ArmConfig.L2_METERS;

    double theta2WithoutOffset = removeOffsetFromTheta2();

		Translation2d effectorPosition = new Translation2d(
			elbowPosition.getX() + (l2 * Math.cos(theta1 + theta2WithoutOffset)) * (armOrientation == ArmOrientation.Front ? 1.0 : -1.0),
			elbowPosition.getY() + (l2 * Math.sin(theta1 + theta2WithoutOffset))
		);

    this.position = effectorPosition;
  }

  public Translation2d getPosition() {
    return this.position;
  }

  public boolean isArmInsideFramePerimeter() {
    return getPosition().getX() < ArmPreset.FRAME_X_BOUNDARY;
  }

  private double applyOffsetToTheta2(double theta1, double theta2) {
    return theta2 + (theta1 - Math.toRadians(90.0));
  }

  private double removeOffsetFromTheta2() {

    double theta1 = Math.toRadians(shoulderJoint.getRelativeAngle());
    double theta2 = Math.toRadians(elbowJoint.getRelativeAngle());

    return theta2 + (Math.toRadians(90.0) - theta1);
  }

  private double lawOfCosines(double a, double b, double c) {
    return Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2)) / (2 * b * c));
  }

  private void initDashboard() {
    DecimalFormat df = new DecimalFormat("#.##");
    df.setMinimumFractionDigits(2);

    var tab = Shuffleboard.getTab("Arm");

    tab.addString("Orientation", () -> this.armOrientation.toString())
      .withPosition(0, 0)
      .withSize(2, 1);

    tab.addString("State", () -> this.armState.toString())
      .withPosition(0, 1)
      .withSize(2, 1);

    tab.addString("X Position", () -> df.format(this.position.getX()))
      .withPosition(0, 2);

    tab.addString("Y Position", () -> df.format(this.position.getY()))
      .withPosition(1, 2);

    addJointToDashboardTab(tab, "J1", shoulderJoint, 2);
    addJointToDashboardTab(tab, "J2", elbowJoint, 3);
    addJointToDashboardTab(tab, "J3", wristJoint, 4);

    tab.addDouble("J2 Without Offset", () -> Math.toDegrees(removeOffsetFromTheta2()))
      .withPosition(3, 4);
  }

  private void addJointToDashboardTab(ShuffleboardTab tab, String jointName, ArmJoint armJoint, int column) {

    tab.addBoolean(jointName + " Initialized", () -> armJoint.isRelativeEncoderInitilized())
      .withPosition(column, 0);

    tab.addDouble(jointName + " Absolute Angle", () -> armJoint.getAbsoulteAngle())
      .withPosition(column, 1);

    tab.addDouble(jointName + " Relative Angle", () -> armJoint.getRelativeAngle())
      .withPosition(column, 2);

    tab.addDouble(jointName + " Target Angle", () -> armJoint.getTargetAngle())
      .withPosition(column, 3);
  }
}