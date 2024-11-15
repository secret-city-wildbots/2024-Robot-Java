package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
//import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Utility.ActuatorInterlocks;

public class Elevator {
    public static boolean stowed = true;

    private final TalonFX elevator = new TalonFX(17, "rio");
    private TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

    private final double elevatorRatio; // from inches to rotations, multiplier
    private double elevatorFeedForward = 0;
    private double elevatorArbitraryFFScalar = 0;
    private double elevatorOutput = 0; // inches

    private PIDController elevatorController = new PIDController(0, 0, 0);

    private MotionMagicConfigs m = elevatorConfig.MotionMagic;

    private final PositionDutyCycle elevatorControlRequest = new PositionDutyCycle(0).withSlot(0);

    public Elevator(double ratio) {
        elevatorRatio = ratio;

        SmartDashboard.putData("Elevator PID Controller", elevatorController);
        SmartDashboard.putNumber("Elevator FF Scalar", elevatorArbitraryFFScalar);
        SmartDashboard.putNumber("Elevator FF Output", elevatorFeedForward);

        elevatorConfig.Slot0.kP = elevatorController.getP();
        elevatorConfig.Slot0.kI = elevatorController.getI();
        elevatorConfig.Slot0.kD = elevatorController.getD();

        m.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        m.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        m.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        elevator.getConfigurator().apply(elevatorConfig);
    }

    public void updateSensors() {
        // gravity * Elevator mass * arbitrary scalar
        elevatorArbitraryFFScalar = SmartDashboard.getNumber("Elevator FF Scalar", 1);
        elevatorFeedForward = 32.17 * 20 * elevatorArbitraryFFScalar;
        SmartDashboard.putNumber("Elevator FF Output", elevatorFeedForward);
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

        double kp = ((PIDController) SmartDashboard.getData("Elevator PID Controller")).getP();
        double ki = ((PIDController) SmartDashboard.getData("Elevator PID Controller")).getI();
        double kd = ((PIDController) SmartDashboard.getData("Elevator PID Controller")).getD();

        if ((elevatorController.getP() != kp) || (elevatorController.getP() != kp)
                || (elevatorController.getP() != kp)) { // only update pid's when needed
            elevatorController.setPID(kp, ki, kd);
            elevatorConfig.Slot0.kP = kp;
            elevatorConfig.Slot0.kI = ki;
            elevatorConfig.Slot0.kD = kd;
            elevator.getConfigurator().apply(elevatorConfig);
        }
    }

    public void updateOutputs() {
        if (!ActuatorInterlocks.isTesting()) {
            elevator.setControl(elevatorControlRequest
                    .withPosition(elevatorOutput * elevatorRatio)
                    .withFeedForward(elevatorFeedForward));
        } else {
            elevator.set(ActuatorInterlocks.TAI_Motors("Elevator_(p)", 0.0));
        }
    }
}
