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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    private double wristArbitraryFFScalar = 0.000001;

    private PIDController wristController = new PIDController(0, 0, 0);

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

    public void updateSensors() {
        rightTemp = right.getDeviceTemp().getValueAsDouble();
        leftTemp = left.getDeviceTemp().getValueAsDouble();
        Dashboard.shooterTemps.set(new double[] { leftTemp, rightTemp });
        rightVelocity = right.getRotorVelocity().getValueAsDouble() * 60;
        leftVelocity = left.getRotorVelocity().getValueAsDouble() * 60;
        Dashboard.shooterVelocities.set(new double[] { leftVelocity, rightVelocity });
        wristStowed = wrist.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
        wristAngle = wrist.getRotorPosition().getValueAsDouble() / 2048 * 360 / wristRatio; // ticks -> degrees
        Dashboard.wristPosition.set(wristAngle);
        wristTemp = wrist.getDeviceTemp().getValueAsDouble();
        Dashboard.wristTemp.set(wristTemp);
        if ((rightVelocity > (0.8 * shooterPower) * 6000)
                && (leftVelocity > (0.8 * shooterPower * shooterRatio) * 6000)) {
            spunUp = true;
        }

        // The sin of the wrist angle * wrist COG * gravity * Wrist mass * arbitrary
        // scalar
        wristArbitraryFFScalar = SmartDashboard.getNumber("Wrist FF Scalar", 1);
        wristFeedForward = Math.sin((wristAngle + 36) / 180 * Math.PI) * 0.5 * 32.17 * 20 * wristArbitraryFFScalar;
        SmartDashboard.putNumber("Wrist FF Output", wristFeedForward);
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
                wristOutput = 70;
                break;
            case CLIMBING:
                wristOutput = 35;
                break;
            case TRAP:
                // idk what to put here yet, it depends on the trap sequence
                break;
        }

        double kp = ((PIDController) SmartDashboard.getData("Wrist PID Controller")).getP();
        double ki = ((PIDController) SmartDashboard.getData("Wrist PID Controller")).getI();
        double kd = ((PIDController) SmartDashboard.getData("Wrist PID Controller")).getD();
        wristController.setPID(kp, ki, kd);
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
        ActuatorInterlocks.TAI_TalonFX_Position(wrist, "Wrist_(p)", wristOutput / 360);

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
