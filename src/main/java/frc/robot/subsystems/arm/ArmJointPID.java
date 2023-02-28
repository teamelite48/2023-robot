package frc.robot.subsystems.arm;

public class ArmJointPID {

    public final double p;
    public final double i;
    public final double d;
    public final double minOutput;
    public final double maxOutput;

    public ArmJointPID(double p, double i, double d, double minOutput, double maxOutput) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }
}