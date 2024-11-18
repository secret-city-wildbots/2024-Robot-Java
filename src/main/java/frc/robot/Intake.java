package frc.robot;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Utility.ActuatorInterlocks;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Intake {
    public static boolean enabled = false;
    private boolean indexing = false;
    private double outerIntakePower;
    private double innerIntakePower;
    private double indexerIntakePower;
    public static boolean bbBroken = false;

    public double innerTemp = -1;
    public double frontTemp = -1;
    public double backTemp = -1;

    private final TalonFX inner = new TalonFX(19, "rio");
    private final TalonFX front = new TalonFX(24, "rio");
    private final TalonFX back = new TalonFX(25, "rio");
    private final CANSparkMax indexer = new CANSparkMax(18, MotorType.kBrushless);
    private final SparkLimitSwitch beamBreak = indexer.getForwardLimitSwitch(Type.kNormallyOpen);

    public Intake(
            double innerPower,
            double outerPower,
            double indexerPower) {
        outerIntakePower = outerPower;
        innerIntakePower = innerPower;
        indexerIntakePower = indexerPower;

        TalonFXConfiguration innerConfig = new TalonFXConfiguration();
        innerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        inner.getConfigurator().apply(innerConfig);

        beamBreak.enableLimitSwitch(true);
    }

    public void updateSensors() {
        bbBroken = beamBreak.isPressed();
        innerTemp = inner.getDeviceTemp().getValueAsDouble();
        frontTemp = front.getDeviceTemp().getValueAsDouble();
        backTemp = back.getDeviceTemp().getValueAsDouble();
    }

    public void updateIntake(XboxController driveController) {
        // Turns off intake and allow indexing when note is picked up
        if (bbBroken) {
            enabled = false;
            beamBreak.enableLimitSwitch(false);
        } else {
            beamBreak.enableLimitSwitch(true);
        }
        if (driveController.getRightBumperPressed()) {
            toggle();
        }
        indexing = (driveController.getLeftTriggerAxis() > 0.7) ||
                (enabled && (!bbBroken)) ||
                (bbBroken && (driveController.getRightTriggerAxis() > 0.7) && Shooter.spunUp);
    }

    public void toggle() {
        if (enabled) {
            enabled = false;
        } else if (!bbBroken) {
            if (Shooter.wristStowed && Elevator.stowed) {
                enabled = true;
            } else {
                enabled = false;
            }
        } else {
            enabled = false;
        }
    }

    public void updateOutputs() {
        ActuatorInterlocks.TAI_TalonFX_Power(inner, "Center_Intake_(p)", (enabled) ? innerIntakePower : 0);
        ActuatorInterlocks.TAI_TalonFX_Power(front, "Outer_Roller_Front_(p)", (enabled) ? outerIntakePower : 0);
        ActuatorInterlocks.TAI_TalonFX_Power(back, "Outer_Roller_Back_(p)", (enabled) ? outerIntakePower : 0);
        ActuatorInterlocks.TAI_SparkMAX_Power(indexer, "Indexer_(p)", (indexing) ? indexerIntakePower : 0);
    }
}
