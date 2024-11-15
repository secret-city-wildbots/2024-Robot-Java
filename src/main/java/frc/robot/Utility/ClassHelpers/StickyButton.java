package frc.robot.Utility.ClassHelpers;

public class StickyButton {
    Timer timer = new Timer();
    Latch latch = new Latch(false);

    public boolean isStuck(boolean buttonPressed, double timeThresh, boolean reset) {
        if (buttonPressed) {
            timer.reset();
        }
        latch.updateLatch(true, false, buttonPressed, reset);
        return (timer.getTimeMillis() <= timeThresh) && latch.latchedBool;
    }
    public boolean isStuck(boolean buttonPressed, double timeThreshMillis) {
        if (buttonPressed) {
            timer.reset();
        }
        latch.updateLatch(true, false, buttonPressed, false);
        return (timer.getTimeMillis() <= timeThreshMillis) && latch.latchedBool;
    }
}
