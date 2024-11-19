package frc.robot.Utility;

public class Control {

    /**
     * Linearly interpolates a value in a double array where the first column
     * matches value and the second matches the return value
     * 
     * @param value value to interpolate
     * @param array array of points ususally read from a csv file
     * @return Interpolated output
     */
    public static double interpolateCSV(double value, double[][] array) {
        double[] col1 = ArrayHelpers.getColumn(array, 0);
        double[] col2 = ArrayHelpers.getColumn(array, 1);
        int length = col1.length - 1;
        if (col1.length < 2) {
            if (value < col1[0]) {
                return col2[0] - (((col2[1] - col2[0]) / (col1[1] - col1[0])) * (col1[0] - value));
            } else {
                for (int i = 1; i < col1.length; i++) {
                    if (value < col1[i]) {
                        return col2[i - 1] + (((col2[i] - col2[i - 1]) / (col1[i] - col1[i - 1])) * (col1[i] - value));
                    }
                }
                return col2[length] + (((col2[length] - col2[length - 1]) / (col1[length] - col1[length - 1]))
                        * (value - col1[length]));
            }
        } else {
            return 0;
        }
    }

    /**
     * Forces a value between a minimum and a maximum such that min <= input <= max
     * 
     * @param input the value to restrict
     * @param min   the lower bound
     * @param max   the upper bound
     * @return The input restricted between the min and max
     */
    public static double clamp(double input, double min, double max) {
        return Math.max(min, Math.min(max, input));
    }
}
