package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.ArmSubsystem.ArmState;

public class ArmPosition {
    public final double x;
    public final double y;
    public final double wristDegrees;
    public final ArmState armState;

    public ArmPosition(double x, double y, double wristDegrees, ArmState armState) {
        this.x = x;
        this.y = y;
        this.wristDegrees = wristDegrees;
        this.armState = armState;
    }
}
