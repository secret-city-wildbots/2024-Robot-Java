package frc.robot.Utility.ClassHelpers;

public class DriverProfile {
    public String profile;
    public double strafeDeadband;
    public double strafeScaling;
    public double strafeMax;
    public double rotateDeadband;
    public double rotateScaling;
    public double rotateMax;
  
    public DriverProfile(String profile, double strafeDeadband, double strafeScaling, double strafeMax,
        double rotateDeadband, double rotateScaling, double rotateMax) {
      this.profile = profile;
      this.strafeDeadband = strafeDeadband;
      this.strafeScaling = strafeScaling;
      this.strafeMax = strafeMax;
      this.rotateDeadband = rotateDeadband;
      this.rotateScaling = rotateScaling;
      this.rotateMax = rotateMax;
    }
  
    public DriverProfile update(String profile, double strafeDeadband, double strafeScaling, double strafeMax,
        double rotateDeadband, double rotateScaling, double rotateMax) {
      this.profile = profile;
      this.strafeDeadband = strafeDeadband;
      this.strafeScaling = strafeScaling;
      this.strafeMax = strafeMax;
      this.rotateDeadband = rotateDeadband;
      this.rotateScaling = rotateScaling;
      this.rotateMax = rotateMax;
      return this;
    }
  }
