package frc.robot.Utility;

public class Control {
    public static double coerce(double input, double min, double max) {
        double output;
        if (input < min) {
            output = min;
        } else if (input > max) {
            output = max;
        } else {
            output = input;
        }
        return output;
    }
}
