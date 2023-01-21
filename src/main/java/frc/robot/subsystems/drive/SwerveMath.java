package frc.robot.subsystems.drive;

public class SwerveMath {

    public final static double PI = Math.PI;
    public final static double TAU = 2.0 * PI;

    public static double normalizeAngle(double angle) {

        angle %= TAU;

        if (angle < 0.0) {
            angle += TAU;
        }

        return angle;
    }

    public static double toRadians(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double toDegrees(double radians) {
        return Math.toDegrees(radians);
    }
}
