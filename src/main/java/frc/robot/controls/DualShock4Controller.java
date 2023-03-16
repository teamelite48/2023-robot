package frc.robot.controls;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DualShock4Controller {

    public Trigger square;
    public Trigger cross;
    public Trigger circle;
    public Trigger triangle;
    public Trigger l1;
    public Trigger r1;
    public Trigger l2;
    public Trigger r2;
    public Trigger share;
    public Trigger options;
    public Trigger l3;
    public Trigger r3;

    public Trigger ps;
    public Trigger touchpad;

    public Trigger up;
    public Trigger right;
    public Trigger down;
    public Trigger left;


    private GenericHID hid;
    private double deadBand = 0.1;
    private boolean squareInputs = true;

    public DualShock4Controller(int port) {

        hid = new GenericHID(port);

        square = new JoystickButton(hid, 1);
        cross = new JoystickButton(hid, 2);
        circle = new JoystickButton(hid, 3);
        triangle = new JoystickButton(hid, 4);
        l1 = new JoystickButton(hid, 5);
        r1 = new JoystickButton(hid, 6);
        l2 = new JoystickButton(hid, 7);
        r2 = new JoystickButton(hid, 8);
        share = new JoystickButton(hid, 9);
        options = new JoystickButton(hid, 10);
        l3 = new JoystickButton(hid, 11);
        r3 = new JoystickButton(hid, 12);

        ps = new JoystickButton(hid, 13);
        touchpad = new JoystickButton(hid, 14);

        up = new Trigger(() -> hid.getPOV() == 0);
        right = new Trigger(() -> hid.getPOV() == 90);
        down = new Trigger(() -> hid.getPOV() == 180);
        left = new Trigger(() -> hid.getPOV() == 270);
    }

    public double getLeftXAxis() {
        return modifyAxis(hid.getRawAxis(0));
    }
    public double getLeftYAxis() {
        return modifyAxis(hid.getRawAxis(1));
    }
    public double getRightXAxis() {
        return modifyAxis(hid.getRawAxis(2));
    }

    public double getRightYAxis() {
         return modifyAxis(hid.getRawAxis(5));
    }

    public Pair<Double, Double> getLeftAxes() {

        double diagonalMovementSpeed = 1.0;

        double horizontalMovement = getLeftXAxis();
        double verticalMovement = getLeftYAxis();

        if (Math.abs(horizontalMovement) > 0.0 && Math.abs(verticalMovement) > 0.0) {

            // Calculate the diagonal movement speed and adjust the horizontal and vertical values accordingly
            double movementMagnitude = Math.sqrt(Math.pow(horizontalMovement, 2.0) + Math.pow(verticalMovement, 2.0));
            double adjustmentFactor = diagonalMovementSpeed / movementMagnitude;
            horizontalMovement = horizontalMovement * adjustmentFactor;
            verticalMovement = verticalMovement * adjustmentFactor;
        }

        return new Pair<Double, Double>(horizontalMovement, verticalMovement);
    }

    private double modifyAxis(Double value) {

        if (Math.abs(value) < deadBand) {
            return 0;
        }

        if (squareInputs == true){
            return Math.abs(value) * Math.abs(value) * value;
        }

        return value;
    }
}
