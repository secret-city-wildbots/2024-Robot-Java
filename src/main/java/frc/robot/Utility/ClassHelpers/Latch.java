package frc.robot.Utility.ClassHelpers;

public class Latch {
    boolean defaultBool;
    boolean latchedBool;

    public Latch(boolean defaultValue) {
        this.defaultBool = defaultValue;
        latchedBool = defaultValue;
    }

    public boolean updateLatch(boolean latchValue, boolean defaultValue, boolean latchSignal, boolean reset) {
        latchedBool = (reset) ? defaultValue : (latchSignal) ? latchValue:latchedBool;
        return latchedBool;
    }

    double defaultDbl;
    double latchedDbl;

    public Latch(double defaultValue) {
        this.defaultDbl = defaultValue;
        latchedDbl = defaultValue;
    }

    public double updateLatch(double latchValue, double defaultValue, boolean latchSignal, boolean reset) {
        latchedDbl = (reset) ? defaultValue : (latchSignal) ? latchValue:latchedDbl;
        return latchedDbl;
    }
}
