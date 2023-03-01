package frc.robot.subsystems.arm;

public class PIDParameters {

    public final double P;
    public final double I;
    public final double D;
    public final double MIN_OUTPUT;
    public final double MAX_OUTPUT;

    public PIDParameters(double p, double i, double d, double minOutput, double maxOutput) {
        this.P = p;
        this.I = i;
        this.D = d;
        this.MIN_OUTPUT = minOutput;
        this.MAX_OUTPUT = maxOutput;
    }
}