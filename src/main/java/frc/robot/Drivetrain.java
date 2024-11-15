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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot.MasterStates;
import frc.robot.Utility.Control;
import frc.robot.Utility.SwerveUtils;
// import frc.robot.Utility.ClassHelpers.DriverProfile;
import frc.robot.Utility.ClassHelpers.Latch;
// import frc.robot.Utility.ClassHelpers.LimitAcceleration;
import frc.robot.Utility.ClassHelpers.StickyButton;

public class Drivetrain {
  public static final double driveGearRatio = 7;
  public static final double azimuthGearRatio = 16;
  public static final double maxGroundSpeed = 16; // ft/s
  public static final double maxRotateSpeed = 360 * (12 * maxGroundSpeed)
      / (2 * Math.PI * (Math.sqrt(Math.pow(Robot.robotLength / 2, 2) + Math.pow(Robot.robotWidth / 2, 2)))); // deg/s

  private final TalonFXConfiguration[] configs = SwerveUtils.swerveModuleConfigs();

  private final NewSwerveModule module0 = new NewSwerveModule(driveGearRatio, azimuthGearRatio, 0, configs[0]);
  private final NewSwerveModule module1 = new NewSwerveModule(driveGearRatio, azimuthGearRatio, 1, configs[1]);
  private final NewSwerveModule module2 = new NewSwerveModule(driveGearRatio, azimuthGearRatio, 2, configs[2]);
  private final NewSwerveModule module3 = new NewSwerveModule(driveGearRatio, azimuthGearRatio, 3, configs[3]);

  private final Translation2d m_module0Location = new Translation2d(0.254, -0.311);
  private final Translation2d m_module1Location = new Translation2d(0.254, 0.311);
  private final Translation2d m_module2Location = new Translation2d(-0.254, 0.311);
  private final Translation2d m_module3Location = new Translation2d(-0.254, -0.311);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_module0Location, m_module1Location,
      m_module2Location, m_module3Location);

  private final Pigeon2 m_pigeon = new Pigeon2(6, "canivore");

  public final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics, m_pigeon.getRotation2d(),
      new SwerveModulePosition[] {
          module0.getPosition(),
          module1.getPosition(),
          module2.getPosition(),
          module3.getPosition()
      });

  public double swerveGroundSpeed = 2;
  private boolean headingLocked = false;
  private final PIDController strafePID = new PIDController(0, 0, 0);

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
    antiDriftPID.enableContinuousInput(0, 360);
    headingAnglePID.enableContinuousInput(0, 360);
  }

  private SwerveModuleState[] moduleStates;

  SlewRateLimiter xAccelerationLimiter = new SlewRateLimiter(1, -1000, 0.0);
  SlewRateLimiter yAccelerationLimiter = new SlewRateLimiter(1, -1000, 0.0);




  public void drive(XboxController driverController, boolean isAutonomous, double period) {

    double[] strafeOutputs = SwerveUtils.swerveScaleStrafe(driverController, isAutonomous);

    if (Math.abs(strafeOutputs[0]) > 0.001 || Math.abs(strafeOutputs[1]) > 0.001) {
      strafeInDeadband = false;
    } else {
      strafeInDeadband = true;
    }

    double rotateOutput = SwerveUtils.swerveScaleRotate(driverController, isAutonomous);

    if (Math.abs(rotateOutput) > 0.001) {
      rotationInDeadband = false;
    } else {
      rotationInDeadband = true;
    }


    double limitedStrafeX = Math.signum(strafeOutputs[0]) * xAccelerationLimiter.calculate(Math.abs(strafeOutputs[0]));
    double limitedStrafeY = Math.signum(strafeOutputs[1]) * yAccelerationLimiter.calculate(Math.abs(strafeOutputs[1]));

    double[] limitedStrafe = new double[]{limitedStrafeX, limitedStrafeY};
      
    double[] assistedStrafe = SwerveUtils.assistStrafe(limitedStrafe, new double[] { Double.NaN, Double.NaN },
        strafePID);

    double assistedRotation = swerveAssistHeading(modeDrivebase(driverController), rotateOutput, limitedStrafe,
        isAutonomous, driverController);

    double[] orientedStrafe = SwerveUtils.fieldOrientedTransform(assistedStrafe, m_pigeon.getAngle() % 360);

    moduleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            orientedStrafe[0], orientedStrafe[1], assistedRotation, m_pigeon.getRotation2d()), period));
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);



    Dashboard.swerve0Details.set(new double[]{
      moduleStates[0].angle.getDegrees(),
      module0.getTemp(),
      moduleStates[0].speedMetersPerSecond,
      0.0 
    });
    Dashboard.swerve1Details.set(new double[]{
      moduleStates[1].angle.getDegrees(),
      module1.getTemp(),
      moduleStates[1].speedMetersPerSecond,
      0.0 
    });
    Dashboard.swerve2Details.set(new double[]{
      moduleStates[2].angle.getDegrees(),
      module2.getTemp(),
      moduleStates[2].speedMetersPerSecond,
      0.0 
    });
    Dashboard.swerve3Details.set(new double[]{
      moduleStates[3].angle.getDegrees(),
      module3.getTemp(),
      moduleStates[3].speedMetersPerSecond,
      0.0 
    });
  }

  private Robot.MasterStates masterState0 = Robot.masterState;

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
  private boolean rotationInDeadband;
  private boolean strafeInDeadband;

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
      rotationInDeadband |= headingAssist;
      return assistedRotation;
    } else {
      headingAssist = true;
      double headingFudgeDeltaT = System.currentTimeMillis() - headingFudgeTime;
      headingFudgeTime = System.currentTimeMillis();
      String currentProfile = Robot.legalDrivers[(int)Dashboard.selectedDriver.get(0.0)];
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
      rotationInDeadband = Math.abs(assistedRotation) < 0.02;
      return (Math.abs(assistedRotation) > 0.02) ? assistedRotation : 0.0;
    }
  }

  
  private boolean[] driveFaults = new boolean[4];
  private boolean[] azimuthFaults = new boolean[4];



  /**
   * 
   * @return A boolean array with the structure:
   *  <ul>
   *    <li> drive faults
   *    <li> azimuth faults
   */
  public boolean[] getFaults() {
    boolean[] faults0 = module0.getSwerveFaults();
    boolean[] faults1 = module1.getSwerveFaults();
    boolean[] faults2 = module2.getSwerveFaults();
    boolean[] faults3 = module3.getSwerveFaults();
    boolean driveFault = faults0[0] || faults1[0] || faults2[0] || faults3[0];
    boolean azimuthFault = faults0[1] || faults1[1] || faults2[1] || faults3[1];
    return new boolean[]{driveFault, azimuthFault};
  }

  
  public void updateOutputs(boolean isAutonomous) {
    boolean[] faults = getFaults();
    boolean fLow = faults[0] || faults[1];
    module0.updateOutputs(moduleStates[0], isAutonomous, fLow);
    module1.updateOutputs(moduleStates[1], isAutonomous, fLow);
    module2.updateOutputs(moduleStates[2], isAutonomous, fLow);
    module3.updateOutputs(moduleStates[3], isAutonomous, fLow);
  }
}
