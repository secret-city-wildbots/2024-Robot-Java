// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;

public class Robot extends TimedRobot {
  public static enum MasterStates {
    STOWED,
    SHOOTING,
    AMP,
    CLIMBING,
    TRAP
  }

  public static MasterStates masterState = MasterStates.STOWED;

  public static double robotLength = 19;
  public static double robotWidth = 23;

  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController m_driverController = new XboxController(0);
  private final XboxController m_manipController = new XboxController(1);
  private final Intake m_intake = new Intake(0.5, 0.5, 0.5);
  private final Shooter m_shooter = new Shooter(0.7, 0.576, 98);
  private final Elevator m_elevator = new Elevator(7.72);
  private final Compressor compressor = new Compressor(2, PneumaticsModuleType.REVPH);

  @SuppressWarnings("unused")
  private final Dashboard m_Dashboard = new Dashboard();

  private final String[] actuatorNames = { "No_Test", "Drive_0_(p)", "Drive_1_(p)", "Drive_2_(p)", "Drive_3_(p)",
      "Azimuth_0_(p)", "Azimuth_1_(p)", "Azimuth_2_(p)", "Azimuth_3_(p)", "Swerve_0_Shifter_(b)",
      "Swerve_1_Shifter_(b)",
      "Swerve_2_Shifter_(b)", "Swerve_3_Shifter_(b)", "Elevator_(p)", "Center_Intake_(p)",
      "Outer_Roller_Front_(p)",
      "Outer_Roller_Back_(p)", "Indexer_(p)", "Shooter_Right_(p)", "Shooter_Left_(p)", "Wrist_(p)" };
  public static final String[] legalDrivers = { "Devin", "Reed", "Driver 3", "Driver 4", "Driver 5", "Programmers",
      "Kidz" };

  public Robot() {
    Dashboard.legalActuatorNames.set(actuatorNames);
    Dashboard.legalDrivers.set(legalDrivers);
  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void autonomousPeriodic() {
    drivetrain.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    // Enable compressor
    compressor.enableAnalog(100, 120);

    // Start by updating all sensor values
    getHighPrioritySensors();

    // Check for any drive updates and drive accordingly
    Pose2d robotPosition = drivetrain.updateOdometry().getPoseMeters();
    drivetrain.drive(m_driverController, isAutonomous(), getPeriod());

    // Check for state updates based on manip inputs
    updateMasterState();

    // Toggle intake if necessary
    m_intake.updateIntake(m_driverController, m_shooter.spunUp);

    // Automatically adjust wrist and shooter based off of master state and
    // controller inputs
    m_shooter.updateWrist(robotPosition);
    m_shooter.updateShooter(m_driverController.getRightTriggerAxis() > 0.2,
        m_driverController.getLeftTriggerAxis() > 0.7, robotPosition, m_intake.bbBroken);

    m_elevator.updateElevator();

    updateOutputs();
  }

  private void getHighPrioritySensors() {
    m_intake.updateSensors();
    m_shooter.updateSensors();
    m_elevator.updateSensors();
  }

  private void updateOutputs() {
    m_intake.updateOutputs();
    m_shooter.updateOutputs();
    m_elevator.updateOutputs();
    drivetrain.updateOutputs(isAutonomous());
  }

  @Override
  public void testPeriodic() {
  }

  public void updateMasterState() {
    /*
     * Change master states to match these manip inputs:
     * Left Bumper: STOW
     * Right Bumper: AMP
     * Right Trigger: SHOOTING
     * Left Trigger & Start Button: CLIMBING
     */
    if (m_manipController.getLeftBumper()) {
      masterState = MasterStates.STOWED;
    } else if (masterState != MasterStates.CLIMBING && m_manipController.getRightBumper()) {
      masterState = MasterStates.AMP;
    } else if (masterState != MasterStates.CLIMBING && m_manipController.getRightTriggerAxis() > 0.7) {
      masterState = MasterStates.SHOOTING;
    } else if (m_manipController.getLeftTriggerAxis() > 0.7 && m_manipController.getStartButton()) {
      masterState = MasterStates.CLIMBING;
    }
  }
}
