package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot.MasterStates;
import frc.robot.Utility.Control;
import frc.robot.Utility.SwerveUtils;
// import frc.robot.Utility.ClassHelpers.DriverProfile;
import frc.robot.Utility.ClassHelpers.Latch;
// import frc.robot.Utility.ClassHelpers.LimitAcceleration;
import frc.robot.Utility.ClassHelpers.StickyButton;

public class Drivetrain {
  public static double driveHighGearRatio;
  public static double driveLowGearRatio;
  public static double azimuthGearRatio;
  public static double maxGroundSpeed; // ft/s
  public static double maxLowGearSpeed; // ft/s
  public static double maxRotateSpeed; // deg/s
  public static double actualWheelDiameter; // inches
  public static double nominalWheelDiameter; // inches

  private final TalonFXConfiguration[] driveConfigs = SwerveUtils.swerveModuleDriveConfigs();
  private final TalonFXConfiguration[] azimuthConfigs = SwerveUtils.swerveModuleAzimuthConfigs();

  private final SwerveModule module0;
  private final SwerveModule module1;
  private final SwerveModule module2;
  private final SwerveModule module3;

  private final Translation2d m_module0Location = new Translation2d(0.254, -0.311);
  private final Translation2d m_module1Location = new Translation2d(0.254, 0.311);
  private final Translation2d m_module2Location = new Translation2d(-0.254, 0.311);
  private final Translation2d m_module3Location = new Translation2d(-0.254, -0.311);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_module0Location, m_module1Location,
      m_module2Location, m_module3Location);

  private final Pigeon2 m_pigeon = new Pigeon2(6, "canivore");

  public final SwerveDriveOdometry m_odometry;

  SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(1, -1000, 0.0);
  SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(1, -1000, 0.0);

  private SwerveModuleState[] moduleStates;

  // Used for modeDrivebase to check if master states changed
  private Robot.MasterStates masterState0 = Robot.masterState;

  public double swerveGroundSpeed = 2;
  private boolean headingLocked = false;
  private final PIDController strafePID = new PIDController(0, 0, 0);

  // Variables stored for the assist heading function
  private StickyButton highSpeedSticky = new StickyButton();
  private boolean headingAssist = false;
  private Latch headingLatch = new Latch(0.0);
  private PIDController antiDriftPID = new PIDController(0, 0, 0);
  private PIDController headingAnglePID = new PIDController(0, 0, 0);
  private boolean headingLatchSignal0 = false;
  private double headingFudgeTime = System.currentTimeMillis();
  private double driverHeadingFudge0 = 0.0;
  private StickyButton noRotationSticky = new StickyButton();
  private boolean lockHeading0 = false;
  private final double headingFudgeMax = 10; // degrees

  public SwerveDriveOdometry updateOdometry() {
    m_odometry.update(
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            module0.getPosition(),
            module1.getPosition(),
            module2.getPosition(),
            module3.getPosition()
        });
    return m_odometry;
  }

  public Drivetrain() {
    // Check for driver profile
    switch (Robot.robotProfile) {
      case "2024_Robot":
        nominalWheelDiameter = 5;
        actualWheelDiameter = 4.53;
        maxGroundSpeed = 18.8 * (actualWheelDiameter / nominalWheelDiameter);
        maxLowGearSpeed = 9.2 * (actualWheelDiameter / nominalWheelDiameter);
        maxRotateSpeed = 360 * (12 * maxGroundSpeed)
            / (2 * Math.PI * (Math.sqrt(Math.pow(Robot.robotLength / 2, 2) + Math.pow(Robot.robotWidth / 2, 2))));
        driveHighGearRatio = 7.13;
        driveLowGearRatio = 14.66;
        azimuthGearRatio = 16;
        break;
      case "Steve2":
        nominalWheelDiameter = 5;
        actualWheelDiameter = 4.78;
        maxGroundSpeed = 17.8 * (actualWheelDiameter / nominalWheelDiameter);
        maxLowGearSpeed = 8.3 * (actualWheelDiameter / nominalWheelDiameter);
        maxRotateSpeed = 360 * (12 * maxGroundSpeed)
            / (2 * Math.PI * (Math.sqrt(Math.pow(Robot.robotLength / 2, 2) + Math.pow(Robot.robotWidth / 2, 2))));
        driveHighGearRatio = 6.42;
        driveLowGearRatio = 14.12;
        azimuthGearRatio = 15.6;
        break;
      default:
        nominalWheelDiameter = 5;
        actualWheelDiameter = 4.78;
        maxGroundSpeed = 17.8 * (actualWheelDiameter / nominalWheelDiameter);
        maxLowGearSpeed = 8.3 * (actualWheelDiameter / nominalWheelDiameter);
        maxRotateSpeed = 360 * (12 * maxGroundSpeed)
            / (2 * Math.PI * (Math.sqrt(Math.pow(Robot.robotLength / 2, 2) + Math.pow(Robot.robotWidth / 2, 2))));
        driveHighGearRatio = 6.42;
        driveLowGearRatio = 14.12;
        azimuthGearRatio = 15.6;
    }

    module0 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 0, driveConfigs[0],
        azimuthConfigs[0]);
    module1 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 1, driveConfigs[1],
        azimuthConfigs[1]);
    module2 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 2, driveConfigs[2],
        azimuthConfigs[2]);
    module3 = new SwerveModule(driveHighGearRatio, driveLowGearRatio, azimuthGearRatio, 3, driveConfigs[3],
        azimuthConfigs[3]);

    m_odometry = new SwerveDriveOdometry(
        m_kinematics, m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            module0.getPosition(),
            module1.getPosition(),
            module2.getPosition(),
            module3.getPosition()
        });

    antiDriftPID.enableContinuousInput(0, 360);
    headingAnglePID.enableContinuousInput(0, 360);
  }

  /**
   * Adjusts joystick outputs based on driver profile, acceleration limits,
   * assisted and locked headings, and driving orientation
   * and stores them in the modulestates[] object
   * 
   * @param driverController
   * @param isAutonomous
   * @param period           How long it has been since the last loop cycle
   */
  public void drive(XboxController driverController, boolean isAutonomous, double period) {
    if (Dashboard.applyProfileSetpoints.get()) {
      double[] setpoints = Dashboard.newDriverProfileSetpoints.get();
      SwerveUtils.updateDriverProfile(setpoints);
      Dashboard.currentDriverProfileSetpoints.set(setpoints);
    }

    // Adjust strafe outputs
    double[] strafeOutputs = SwerveUtils.swerveScaleStrafe(driverController, isAutonomous);
    double limitedStrafeX = Math.signum(strafeOutputs[0]) * xAccelerationLimiter.calculate(Math.abs(strafeOutputs[0]));
    double limitedStrafeY = Math.signum(strafeOutputs[1]) * yAccelerationLimiter.calculate(Math.abs(strafeOutputs[1]));

    double[] limitedStrafe = new double[] { limitedStrafeX, limitedStrafeY };

    double[] assistedStrafe = SwerveUtils.assistStrafe(limitedStrafe, new double[] { Double.NaN, Double.NaN },
        strafePID);
    double[] orientedStrafe = SwerveUtils.fieldOrientedTransform(assistedStrafe, m_pigeon.getAngle() % 360);

    // Adjust rotate outputs
    double rotateOutput = SwerveUtils.swerveScaleRotate(driverController, isAutonomous);

    double assistedRotation = swerveAssistHeading(modeDrivebase(driverController), rotateOutput, limitedStrafe,
        isAutonomous, driverController);

    // Store information in modulestates
    moduleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            orientedStrafe[0], orientedStrafe[1], assistedRotation, m_pigeon.getRotation2d()), period));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

    moduleStates[0].speedMetersPerSecond = SwerveUtils.driveCommandToPower(moduleStates[0],
        module0.shifter.get() == Value.kForward);
    moduleStates[1].speedMetersPerSecond = SwerveUtils.driveCommandToPower(moduleStates[1],
        module1.shifter.get() == Value.kForward);
    moduleStates[2].speedMetersPerSecond = SwerveUtils.driveCommandToPower(moduleStates[2],
        module2.shifter.get() == Value.kForward);
    moduleStates[3].speedMetersPerSecond = SwerveUtils.driveCommandToPower(moduleStates[3],
        module3.shifter.get() == Value.kForward);

    // Report to the dashboard
    Dashboard.swerve0Details.set(new double[] {
        moduleStates[0].angle.getDegrees(),
        module0.getTemp(),
        moduleStates[0].speedMetersPerSecond,
        (module0.shifter.get() == Value.kForward) ? 1 : 0
    });
    Dashboard.swerve1Details.set(new double[] {
        moduleStates[1].angle.getDegrees(),
        module1.getTemp(),
        moduleStates[1].speedMetersPerSecond,
        (module1.shifter.get() == Value.kForward) ? 1 : 0
    });
    Dashboard.swerve2Details.set(new double[] {
        moduleStates[2].angle.getDegrees(),
        module2.getTemp(),
        moduleStates[2].speedMetersPerSecond,
        (module2.shifter.get() == Value.kForward) ? 1 : 0
    });
    Dashboard.swerve3Details.set(new double[] {
        moduleStates[3].angle.getDegrees(),
        module3.getTemp(),
        moduleStates[3].speedMetersPerSecond,
        (module3.shifter.get() == Value.kForward) ? 1 : 0
    });
  }

  /**
   * 
   * @param driverController
   * @return
   */
  private double modeDrivebase(XboxController driverController) {
    if ((masterState0 != Robot.masterState) || (driverController.getYButton())) {
      headingLocked = true;
    } else if (driverController.getXButton()) {
      headingLocked = false;
    }
    boolean red = DriverStation.getAlliance().get() == Alliance.Red;
    switch (Robot.masterState) {
      case STOWED:
        headingLocked = false;
        masterState0 = MasterStates.STOWED;
        return Double.NaN;
      case SHOOTING:
        double angle;
        switch (Shooter.state) {
          case SUB:
            angle = Double.NaN;
            break;
          case TACO:
            angle = -1 * Math.toDegrees(
                Math.atan2(((red) ? 107 : 212) - m_odometry.getPoseMeters().getY(), m_odometry.getPoseMeters().getX()));
            break;
          case LOB:
            angle = -1 * Math.toDegrees(Math.atan2(((red) ? 28.5 : 331.5) - m_odometry.getPoseMeters().getY(),
                m_odometry.getPoseMeters().getX()));
          default:
            angle = Double.NaN;
        }
        masterState0 = MasterStates.SHOOTING;
        return (headingLocked) ? angle : Double.NaN;
      case AMP:
        masterState0 = MasterStates.AMP;
        return (headingLocked) ? ((red) ? 90 : 270) : Double.NaN;
      case CLIMBING:
        masterState0 = MasterStates.CLIMBING;
        return Double.NaN;
      case TRAP:
        masterState0 = MasterStates.TRAP;
        return Double.NaN;
      default:
        return Double.NaN;
    }
  }

  /**
   * Adjusts joystick outputs to help with angle drift and heading locking
   * 
   * @param lockedHeading    If the heading is locked
   * @param joystickRotation
   * @param limitedStrafe
   * @param isAutonomous
   * @param driverController
   * @return Locked or assisted heading output
   */
  private double swerveAssistHeading(double lockedHeading, double joystickRotation, double[] limitedStrafe,
      boolean isAutonomous, XboxController driverController) {
    double pigeonAngle = m_pigeon.getAngle() % 360;
    if (lockedHeading != lockedHeading) {
      lockHeading0 = false;
      boolean disableHeadingAssist = false;
      boolean lowSpeed = !highSpeedSticky.isStuck(swerveGroundSpeed > 1, 250);
      disableHeadingAssist |= lowSpeed; // Driving at low speed
      disableHeadingAssist |= (Math.max(Math.abs(limitedStrafe[0]), Math.abs(limitedStrafe[1]))) <= 0.001; // Driver not
                                                                                                           // driving
      disableHeadingAssist |= Math.abs(joystickRotation) >= 0.001; // Driver trying to rotate
      disableHeadingAssist |= DriverStation.isDisabled(); // Robot disable
      disableHeadingAssist |= isAutonomous; // Robot autonomous
      headingAssist = !disableHeadingAssist;

      if (!headingAssist) {
        return joystickRotation;
      }

      boolean headingLatchSignal = (noRotationSticky.isStuck(Math.abs(joystickRotation) <= 0.001, 100.0)
          && !(DriverStation.isDisabled() || isAutonomous || lowSpeed));

      antiDriftPID.setP(swerveGroundSpeed * antiDriftPID.getP()); // increase kp base on ground velocity
      double assistedRotation = antiDriftPID.calculate(
          pigeonAngle,
          headingLatch.updateLatch(pigeonAngle, pigeonAngle,
              (!headingLatchSignal0) && headingLatchSignal,
              !headingLatchSignal));
      headingLatchSignal0 = headingLatchSignal;
      return assistedRotation;
    } else {
      headingAssist = true;
      double headingFudgeDeltaT = System.currentTimeMillis() - headingFudgeTime;
      headingFudgeTime = System.currentTimeMillis();
      String currentProfile = Robot.legalDrivers[(int) Dashboard.selectedDriver.get(0.0)];
      double driverHeadingFudge;
      if (driverController.getBButton() || driverController.getYButton() || lockHeading0 == false) {
        driverHeadingFudge = 0.0;
        driverHeadingFudge0 = 0.0;
      } else {
        driverHeadingFudge = headingFudgeDeltaT * maxRotateSpeed
            * SwerveUtils.readDriverProfiles(currentProfile).rotateMax * joystickRotation;
        driverHeadingFudge0 += driverHeadingFudge;
      }
      driverHeadingFudge0 = Control.clamp(driverHeadingFudge0, headingFudgeMax, -1 * headingFudgeMax);
      lockHeading0 = true;
      double assistedRotation = headingAnglePID.calculate(pigeonAngle, lockedHeading + driverHeadingFudge0);
      return (Math.abs(assistedRotation) > 0.02) ? assistedRotation : 0.0;
    }
  }

  private boolean[] driveFaults = new boolean[4];
  private boolean[] azimuthFaults = new boolean[4];

  /**
   * 
   * @return A boolean array with the structure:
   *         <ul>
   *         <li>drive faults
   *         <li>azimuth faults
   */
  public boolean[] getFaults() {
    boolean[] faults0 = module0.getSwerveFaults();
    boolean[] faults1 = module1.getSwerveFaults();
    boolean[] faults2 = module2.getSwerveFaults();
    boolean[] faults3 = module3.getSwerveFaults();
    driveFaults = new boolean[] { faults0[0], faults1[0], faults2[0], faults3[0] };
    azimuthFaults = new boolean[] { faults0[1], faults1[1], faults2[1], faults3[1] };
    boolean driveFault = faults0[0] || faults1[0] || faults2[0] || faults3[0];
    boolean azimuthFault = faults0[1] || faults1[1] || faults2[1] || faults3[1];
    return new boolean[] { driveFault, azimuthFault };
  }

  /**
   * Sends stored modulestate outputs to each individual module
   * 
   * @param isAutonomous
   */
  public void updateOutputs(boolean isAutonomous) {
    boolean[] faults = getFaults();
    boolean fLow = faults[0] || faults[1];
    boolean homeWheels = Dashboard.homeWheels.get();
    module0.updateOutputs(moduleStates[0], isAutonomous, fLow, driveFaults[0] || azimuthFaults[0], homeWheels);
    module1.updateOutputs(moduleStates[1], isAutonomous, fLow, driveFaults[1] || azimuthFaults[1], homeWheels);
    module2.updateOutputs(moduleStates[2], isAutonomous, fLow, driveFaults[2] || azimuthFaults[2], homeWheels);
    module3.updateOutputs(moduleStates[3], isAutonomous, fLow, driveFaults[3] || azimuthFaults[3], homeWheels);
  }
}
