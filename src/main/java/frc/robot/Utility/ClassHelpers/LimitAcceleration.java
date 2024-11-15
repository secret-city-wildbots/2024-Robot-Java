package frc.robot.Utility.ClassHelpers;

public class LimitAcceleration {
    public boolean applyLimit0 = false;
    public double rawInput0 = 0;
    public double time0 = (double)System.currentTimeMillis();

    public double updateLimit(double rawInput, boolean applyLimit, double accelerationLimit, double decelerationLimit) {
        double deltaT = ((double)System.currentTimeMillis() - time0) / 1000.0;
        time0 = (double)System.currentTimeMillis();
        if (applyLimit != this.applyLimit0) {
            rawInput0 = rawInput;
        }
        double maxDeltaVelocity = deltaT * ((rawInput>0) ? accelerationLimit : decelerationLimit);
        double deltaVelocity = Math.min(Math.abs(rawInput - rawInput0), maxDeltaVelocity);
        rawInput0 += deltaVelocity;
        double output = (applyLimit) ? rawInput0 : rawInput;
        applyLimit0 = applyLimit;
        return output;
    }
}
