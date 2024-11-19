package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Utility.ActuatorInterlocks;

public class Elevator {
    public static boolean stowed = true;
    public static boolean climbed = false;

    private double height = 0;
    private double motorTemp = 0;

    private final TalonFX elevator = new TalonFX(17, "rio");
    private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    private final double elevatorRatio; // from inches to rotations, multiplier
    private final double elevatorFeedForward;
    private double elevatorArbitraryFFScalar = 0;
    private double elevatorOutput = 0; // inches
    private final double allowedError = 0.2; // inches

    private PIDController elevatorController = new PIDController(0, 0, 0);

    private MotionMagicConfigs motionMagicConfigs = elevatorConfig.MotionMagic;

    private final PositionDutyCycle elevatorControlRequest = new PositionDutyCycle(0).withSlot(0);

    private boolean unlockElevator0 = false;

    public Elevator() {
        switch (Robot.robotProfile) {
            case "2024_Robot":
                elevatorRatio = 7.72;
                break;
            case "Steve2":
                elevatorRatio = 7.72;
                break;
            default:
                elevatorRatio = 7.72;
        }
        

        elevatorConfig.Slot0.kP = elevatorController.getP();
        elevatorConfig.Slot0.kI = elevatorController.getI();
        elevatorConfig.Slot0.kD = elevatorController.getD();

        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        elevator.getConfigurator().apply(elevatorConfig);

        // gravity * mass * arbitrary FF scalar
        elevatorFeedForward = 32.17 * 20 * elevatorArbitraryFFScalar;
    }

    public void updateSensors() {     
        height = elevator.getPosition().getValueAsDouble() / elevatorRatio;
        Dashboard.elevatorPosition.set(height);
        motorTemp = elevator.getDeviceTemp().getValueAsDouble();
        Dashboard.elevatorTemp.set(motorTemp);
        climbed = Math.abs(height - elevatorOutput) < allowedError;
    }

    public void updateElevator() {
        switch (Robot.masterState) {
            case STOWED:
            case SHOOTING:
                elevatorOutput = 0; // inches
                break;
            case AMP:
                elevatorOutput = 5;
                break;
            case CLIMBING:
                elevatorOutput = 5;
                break;
            case TRAP:
                // idk what to put here yet, it depends on the trap sequence
                break;
        }
    }

    public void updateOutputs() {
        if (!ActuatorInterlocks.isTesting()) {
            elevator.setControl(elevatorControlRequest
                    .withPosition(elevatorOutput * elevatorRatio)
                    .withFeedForward(elevatorFeedForward));
        } else {
            ActuatorInterlocks.TAI_TalonFX_Power(elevator, "Elevator_(p)", 0.0);
        }

        // Put elevator in coast while unlocked and only when changed
        boolean unlockElevator = Dashboard.unlockElevator.get();
        if (unlockElevator!=unlockElevator0) {
            if (unlockElevator) {
                elevator.setNeutralMode(NeutralModeValue.Coast);
            } else {
                elevator.setNeutralMode(NeutralModeValue.Brake);
            }
        }
        unlockElevator0 = unlockElevator;
    }
}
