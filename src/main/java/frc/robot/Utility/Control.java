package frc.robot.Utility;

public class Control {


    
    /**
     * Forces a value between a minimum and a maximum such that min <= input <= max
     * @param input the value to restrict
     * @param min the lower bound
     * @param max the upper bound
     * @return The input restricted between the min and max
     */
    public static double clamp(double input, double min, double max) {
        return Math.max(min, Math.min(max, input));
    }
}
