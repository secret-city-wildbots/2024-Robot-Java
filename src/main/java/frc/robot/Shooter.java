package frc.robot;

import frc.robot.Utility.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
    public double rightVelocity = -1;
    public double leftVelocity = -1;
    public double wristAngle = -1;

    public boolean spunUp = false;

    private double wristOutput = -1; // degrees

    private final TalonFX wrist = new TalonFX(14, "rio");
    private final TalonFX right = new TalonFX(15, "rio");
    private final TalonFX left = new TalonFX(16, "rio");

    private TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    private TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    private TalonFXConfiguration leftConfig = new TalonFXConfiguration();

    private double wristFeedForward = 0;
    private double wristArbitraryFFScalar = 0.000001;

    private PIDController wristController = new PIDController(0, 0, 0);

    private final double wristRatio;

    public Shooter(
            double shootPower,
            double shooterWheelRatio,
            double wristGearRatio) {
        shooterPower = shootPower;
        shooterRatio = shooterWheelRatio;
        wristRatio = wristGearRatio;

        wristConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        wrist.getConfigurator().apply(wristConfig);

        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        right.getConfigurator().apply(rightConfig);

        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        left.getConfigurator().apply(leftConfig);

        SmartDashboard.putData("Wrist PID Controller", wristController);
        SmartDashboard.putNumber("Wrist FF Scalar", wristArbitraryFFScalar);
        SmartDashboard.putNumber("Wrist FF Output", wristFeedForward);

        wrist.setPosition(0);
    }

    public void updateSensors() {
        rightTemp = right.getDeviceTemp().getValueAsDouble();
        leftTemp = left.getDeviceTemp().getValueAsDouble();
        rightVelocity = right.getRotorVelocity().getValueAsDouble() * 60;
        leftVelocity = left.getRotorVelocity().getValueAsDouble() * 60;
        wristStowed = wrist.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
        wristAngle = wrist.getRotorPosition().getValueAsDouble() / 2048 * 360 / wristRatio; // ticks -> degrees
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

    public void updateWrist(Pose2d robotPosition) {
        switch (Robot.masterState) {
            case STOWED:
                wristOutput = 0; // degrees
                break;
            case SHOOTING:
                switch (Shooter.state) {
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
        return interpolateWristAngle(robotPosition.getY(), wristCalibrations);
    }

    private double interpolateWristAngle(double value, double[][] array) {
        double[] col1 = ArrayHelpers.getColumn(array, 0);
        double[] col2 = ArrayHelpers.getColumn(array, 1);
        int length = col1.length - 1;
        if (col1.length < 2) {
            if (value < col1[0]) {
                return col2[0] - (((col2[1] - col2[0]) / (col1[1] - col1[0])) * (col1[0] - value));
            } else {
                for (int i = 1; i < col1.length; i++) {
                    if (value < col1[i]) {
                        return col2[i - 1] + (((col2[i] - col2[i - 1]) / (col1[i] - col1[i - 1])) * (col1[i] - value));
                    }
                }
                return col2[length] + (((col2[length] - col2[length - 1]) / (col1[length] - col1[length - 1]))
                        * (value - col1[length]));
            }
        } else {
            return 0;
        }
    }

    public void updateOutputs() {
        // System.out.println(wrist.getRotorPosition().getValueAsDouble()*360/wristRatio);
        // System.out.println(wristOutput);
        right.set(ActuatorInterlocks.TAI_Motors("Shooter_Right_(p)", (spin) ? shooterPower : 0));
        left.set(ActuatorInterlocks.TAI_Motors("Shooter_Left_(p)", (spin) ? shooterPower * shooterRatio : 0));
        wrist.set(ActuatorInterlocks.TAI_Motors("Wrist_(p)", wristFeedForward + wristController
                .calculate(wrist.getRotorPosition().getValueAsDouble() * 360 / wristRatio, wristOutput)));
    }
}
