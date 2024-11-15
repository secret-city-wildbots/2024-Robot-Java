package frc.robot.Utility.ClassHelpers;

public class Timer {
    private double time0 = (double)System.currentTimeMillis();

    public double getTimeMillis() {
        return (double)(System.currentTimeMillis()) - time0;
    }
    public void reset() {
        time0 = (double)System.currentTimeMillis();
    }
}
