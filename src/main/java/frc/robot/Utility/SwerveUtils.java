package frc.robot.Utility;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Dashboard;
import frc.robot.Robot;
import frc.robot.Utility.ClassHelpers.DriverProfile;

public class SwerveUtils {



  static DriverProfile activeDriverProfile = new DriverProfile("", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);



  /**
   * Reads in driver profile by accessing the calibration file matching the input profile 
   * and parses the file settings into a Driverprofile object
   * @param profile 
   * @return
   */
  public static DriverProfile readDriverProfiles(String profile) {
    // If driver profile was most recently checked, skip the rest of this function
    if (activeDriverProfile.profile.equals(profile)) {
      return activeDriverProfile;
    }

    // Read in driver profile xml file, split into sections, and remove the inital setup before the first <Val> tag
    String file = FileHelpers.readFile("/home/lvuser/calibrations/" + profile);
    String[] profileSetpoints = file.split("<Val>");
    profileSetpoints[0] = "";

    //For each non "" element of the array, parse in the value shown in the first 13 characters of the substring
    double[] outputArray = new double[6];

    int i = 0;
    for (String x : profileSetpoints) {
      if (!x.equals("")) {
        outputArray[i] = Double.parseDouble(x.substring(0, 12));
        i += 1;
      }
    }

    //Store the values in a DriverProfile object
    DriverProfile output = new DriverProfile(profile, outputArray[0], outputArray[1], outputArray[2], outputArray[3],
        outputArray[4], outputArray[5]);
    
    return output;
  }



  /**
   * Creates an array of configurations for TalonFXs with the following changes from default:
   * <ul>
   *  <li> Rotation directions are specified based on module position
   *  <li> Forward and reverse limits are disabled due to the shifter sensors being plugged into these ports in some versions
   *  <li> Neutral mode is set to brake so that the robot stops when prompted
   *  <li> Open and closed loop ramps are both 0.1 to get consistent acceleration decceleration without drawing too much current
   * </ul>
   * 
   * Where element 0 matches module 0, element 1 matches module 1, etc...
   * @return
   */
  public static TalonFXConfiguration[] swerveModuleConfigs() {
    TalonFXConfiguration[] configs = new TalonFXConfiguration[4];

    // Front right
    configs[0] = new TalonFXConfiguration();
    configs[0].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[0].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[0].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[0].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[0].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    configs[0].MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Front Left
    configs[1] = new TalonFXConfiguration();
    configs[1].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[1].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[1].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[1].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[1].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    configs[1].MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Back left
    configs[2] = new TalonFXConfiguration();
    configs[2].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[2].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[2].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[2].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[2].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    configs[2].MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Back right
    configs[3] = new TalonFXConfiguration();
    configs[3].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[3].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[3].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[3].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[3].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
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



  /**
   * Scales and caps raw right joystick output based on current selected driver profile
   * settings
   * 
   * @param driverController
   * @param isAutonomous
   * @return Scaled rotate joystick command
   */
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



  /**
   * Adjusts the orientation of the joystick outputs to be field oriented instead of robot oriented
   * @param joysticks
   * @param heading
   * @return 1d double array of joystick values for x and y outputs
   */
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