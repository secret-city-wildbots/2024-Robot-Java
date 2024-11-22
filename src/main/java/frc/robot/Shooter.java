package frc.robot;

import frc.robot.Utility.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;

public class Shooter {

    enum ShooterStates {
        SUB,
        TACO,
        LOB
    }

    private double[][] wristCalibrations = FileHelpers.parseCSV("/home/lvuser/calibrations/WristAngleByDistance.csv");

    public static ShooterStates state = ShooterStates.SUB;

    public static boolean wristStowed = true;
    private double shooterPower;
    private double shooterRatio;
    private boolean spin = false;

    public double rightTemp = -1;
    public double leftTemp = -1;
    public double wristTemp = -1;
    public double rightVelocity = -1;
    public double leftVelocity = -1;
    public double wristAngle = -1;

    public static boolean spunUp = false;

    private double wristOutput = -1; // degrees

    private final TalonFX wrist = new TalonFX(14, "rio");
    private final TalonFX right = new TalonFX(15, "rio");
    private final TalonFX left = new TalonFX(16, "rio");

    private TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    private Slot0Configs wristPIDConfigs = new Slot0Configs();
    private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

    private double wristFeedForward = 0;
    private double wristArbitraryFFScalar = 0; // Currently disabled for testing PIDs

    private PIDController wristController = new PIDController(0.11, 0.015, 0.025);

    private final double wristRatio;

    private boolean unlockWrist0 = false;

    public Shooter(
            double shootPower,
            double shooterWheelRatio) {

        switch (Robot.robotProfile) {
            case "2024_Robot":
                wristRatio = 98;
                break;
            case "Steve2":
                wristRatio = 98;
                break;
            default:
                wristRatio = 98;
        }

        shooterPower = shootPower;
        shooterRatio = shooterWheelRatio;

        wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wristPIDConfigs.kP = wristController.getP();
        wristPIDConfigs.kI = wristController.getI();
        wristPIDConfigs.kD = wristController.getD();
        wrist.getConfigurator().apply(wristConfig);
        wrist.getConfigurator().apply(wristPIDConfigs);

        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        right.getConfigurator().apply(rightConfig);

        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        left.getConfigurator().apply(leftConfig);
        
        wrist.setPosition(0);
    }

    /**
     * Retreives all sensor values for the shooter motors and wrist to use for other
     * functions
     */
    public void updateSensors() {
        // Read in motor sensor values (velocity, temperature, and limit switches)
        rightTemp = right.getDeviceTemp().getValueAsDouble();
        leftTemp = left.getDeviceTemp().getValueAsDouble();

        rightVelocity = right.getRotorVelocity().getValueAsDouble() * 60;
        leftVelocity = left.getRotorVelocity().getValueAsDouble() * 60;

        wristStowed = wrist.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
        wristAngle = wrist.getRotorPosition().getValueAsDouble() * 360 / wristRatio; // rotations -> degrees

        wristTemp = wrist.getDeviceTemp().getValueAsDouble();

        // Decide whether or not the shooter wheels are spun up enough
        if ((rightVelocity > (0.8 * shooterPower) * 6000)
                && (leftVelocity > (0.8 * shooterPower * shooterRatio) * 6000)) {
            spunUp = true;
        }

        // Report values to the dashboard
        Dashboard.shooterTemps.set(new double[] { leftTemp, rightTemp });
        Dashboard.shooterVelocities.set(new double[] { leftVelocity, rightVelocity });
        Dashboard.wristPosition.set(wristAngle);
        Dashboard.wristTemp.set(wristTemp);

        // The sin of the wrist angle * wrist COG * gravity * Wrist mass * arbitrary
        // scalar
        wristFeedForward = Math.sin((wristAngle + 36) / 180 * Math.PI) * 0.5 * 32.17 * 20 * wristArbitraryFFScalar;        
    }

    public void updateWrist(Pose2d robotPosition, XboxController manipController) {
        if (manipController.getYButtonPressed()) {
            state = ShooterStates.TACO;
        } else if (manipController.getXButtonPressed()) {
            state = ShooterStates.SUB;
        } else if (manipController.getAButtonPressed()) {
            state = ShooterStates.LOB;
        }

        Dashboard.shooterState.set(state.ordinal());

        switch (Robot.masterState) {
            case STOWED:
                wristOutput = 0; // degrees
                break;
            case SHOOTING:
                switch (state) {
                    case SUB:
                        wristOutput = 0;
                        break;
                    case TACO:
                        wristOutput = calculateWristAngle(robotPosition);
                        break;
                    case LOB:
                        wristOutput = 0;
                        break;
                }
                break;
            case AMP:
                wristOutput = 56;
                break;
            case CLIMBING:
                wristOutput = 35;
                break;
            case TRAP:
                // idk what to put here yet, it depends on the trap sequence
                break;
        }
    }

    public void updateShooter(boolean rightTrigger, boolean leftTrigger, Pose2d robotPosition, boolean haveNote) {
        if (leftTrigger || rightTrigger) {
            spin = true;
        } else if (Robot.masterState == Robot.MasterStates.AMP) {
            spin = true;
        } else if (robotPosition.getY() * 39.37 < 312 && haveNote) {
            spin = true;
        } else {
            spin = false;
        }
    }

    private double calculateWristAngle(Pose2d robotPosition) {
        return Control.interpolateCSV(robotPosition.getY(), wristCalibrations);
    }

    public void updateOutputs() {
        ActuatorInterlocks.TAI_TalonFX_Power(right, "Shooter_Right_(p)", (spin) ? shooterPower : 0);
        ActuatorInterlocks.TAI_TalonFX_Power(left, "Shooter_Left_(p)", (spin) ? shooterPower * shooterRatio : 0);
        ActuatorInterlocks.TAI_TalonFX_Position(wrist, "Wrist_(p)", wristOutput / 360 * wristRatio, wristFeedForward);

        // Put Wrist in coast while unlocked and only when changed
        boolean unlockWrist = Dashboard.unlockWrist.get();
        if (unlockWrist != unlockWrist0) {
            if (unlockWrist) {
                wrist.setNeutralMode(NeutralModeValue.Coast);
            } else {
                wrist.setNeutralMode(NeutralModeValue.Brake);
            }
        }
        unlockWrist0 = unlockWrist;
    }
}
