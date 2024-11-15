package frc.robot.Utility;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Utility.ClassHelpers.DriverProfile;

public class SwerveUtils {
  static DriverProfile activeDriverProfile = new DriverProfile("", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  public static DriverProfile readDriverProfiles(String profile) {
    if (activeDriverProfile.profile.equals(profile)) {
      return activeDriverProfile;
    }
    String file = FileHelpers.readFile("/home/lvuser/calibrations/" + profile);
    String[] profileSetpoints = file.split("<Val>");
    profileSetpoints[0] = "";

    double[] outputArray = new double[6];

    int i = 0;
    for (String x : profileSetpoints) {
      if (!x.equals("")) {
        outputArray[i] = Double.parseDouble(x.substring(0, 12));
        i += 1;
      }
    }

    DriverProfile output = new DriverProfile(profile, outputArray[0], outputArray[1], outputArray[2], outputArray[3],
        outputArray[4], outputArray[5]);
    return output;
  }

  public static TalonFXConfiguration[] swerveModuleConfigs() {
    /*
     * configs will be an array of TalonFX Configurations (4 total - 1 for each
     * swerve module)
     * 
     * Note: you will see that we have disabled on the drive motors the following:
     * - ForwardLimitEnable
     * - ReverseLimitEnable
     * This is because we have the shifters sensors connected to the motor
     * controller input.
     */
    TalonFXConfiguration[] configs = new TalonFXConfiguration[4];

    /*
     * Swerve Module Drive Motor Configs (Front Right)
     * 
     * Note: We need to set the motor output to be CCW (counter clockwise), becuase
     * we have the bevel
     * gears facing inwards.
     */
    configs[0] = new TalonFXConfiguration();
    configs[0].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[0].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[0].MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    /*
     * Swerve Module Drive Motor Configs (Front Left)
     * 
     * Note: We need to set the motor output to be CW (clockwise), because we have
     * the bevel gears facing
     * inwards.
     */
    configs[1] = new TalonFXConfiguration();
    configs[1].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[1].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[1].MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /*
     * Swerve Module Drive Motor Configs (Back Left)
     * 
     * Note: We need to set the motor output to be CW (clockwise), because we have
     * the bevel gears facing
     * inwards.
     */
    configs[2] = new TalonFXConfiguration();
    configs[2].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[2].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[2].MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    /*
     * Swerve Module Drive Motor Configs (Back Right)
     * 
     * Note: We need to set the motor output to be CCW (counter clockwise), becuase
     * we have the bevel
     * gears facing inwards.
     */
    configs[3] = new TalonFXConfiguration();
    configs[3].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[3].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[3].MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    return configs;
  }

  /**
   * Scales and caps raw joystick outputs based on current selected driver profile
   * settings
   * 
   * @param driverController
   * @param isAutonomous
   * 
   * @return Scaled joystick outputs
   */
  public static double[] swerveScaleStrafe(XboxController driverController, boolean isAutonomous) {

    // Getting driver profile settings
    String profile = Robot.legalDrivers[(int) Dashboard.selectedDriver.get(0.0)];
    DriverProfile currentProfile = SwerveUtils.readDriverProfiles(profile);
    double deadband = (isAutonomous) ? 0.01 : currentProfile.strafeDeadband;
    double strafeScaling = ((isAutonomous) ? 1 : currentProfile.strafeScaling);
    double strafeMax = ((isAutonomous) ? 1 : currentProfile.strafeMax);

    // Negate and swap raw joystick outputs to work with FRC field orientation
    double rawX = -1 * driverController.getLeftY();
    double rawY = -1 * driverController.getLeftX();

    // Disable joystick outputs while within deadband
    double joystickSaturation = Math.sqrt((rawX * rawX) + (rawY * rawY));
    if (joystickSaturation <= deadband) {
      return new double[] { 0.0, 0.0 };
    }

    // Sanitize joystick saturation (insure it isn't more than 1 and prevent
    // dividing by 0)
    double joystickRange;
    if (joystickSaturation > 1.0) {
      joystickRange = 1;
    } else {
      joystickRange = (joystickSaturation >= 0.01) ? joystickSaturation : 0.01;
      joystickSaturation = 1;
    }

    double exponentialScalar = Math.pow((joystickRange - deadband) / (1 - deadband), strafeScaling)
        / joystickRange;

    // Normalize raw X and Y if saturation is >1 and increase joystick outputs
    // exponentially based on strafeScaling and clamp values below strafeMax
    return new double[] { rawX / joystickSaturation * exponentialScalar * strafeMax,
        rawY / joystickSaturation * exponentialScalar * strafeMax };
  }

  public static double swerveScaleRotate(XboxController driverController, boolean isAutonomous) {
    String profile = Robot.legalDrivers[(int) Dashboard.selectedDriver.get(0.0)];
    DriverProfile currentProfile = SwerveUtils.readDriverProfiles(profile);
    double deadband = (isAutonomous) ? 0.01 : currentProfile.rotateDeadband;
    double rawY = -driverController.getRightX();
    double joystickSaturation = Math.abs(rawY);
    if (joystickSaturation <= deadband) {
      return 0.0;
    }
    double exponentialScalar = Math.pow((joystickSaturation - deadband) / (1 - deadband),
        ((isAutonomous) ? 1 : currentProfile.rotateScaling));
    return Math.signum(rawY) * exponentialScalar * ((isAutonomous) ? 1 : currentProfile.rotateMax);
  }

  /**
   * This funciton is not complete and needs mto be made
   * but i cant be bothered to do it rn
   */
  public static double[] assistStrafe(double[] joysticks, double[] lockedXY, PIDController strafePID) {
    return joysticks;
  }

  public static double[] fieldOrientedTransform(double[] joysticks, double heading) {
    double[] output = new double[] { 0.0, 0.0 };
    double x = joysticks[0];
    double y = joysticks[1];
    double headingR = Math.toRadians(heading);
    double cos = Math.cos(headingR);
    double sin = Math.sin(headingR);
    output[0] = (x * cos) + (y * sin);
    output[1] = (y * cos) - (x * sin);

    return output;
  }
}