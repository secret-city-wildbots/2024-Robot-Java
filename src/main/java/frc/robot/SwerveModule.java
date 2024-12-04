package frc.robot;

import frc.robot.Utility.ActuatorInterlocks;
import frc.robot.Utility.ClassHelpers.Timer;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class SwerveModule {
    enum ShiftedStates {
        LOW,
        TRANS,
        HIGH
    }

    private final double shiftToHighRPM = 4000;
    private final double shiftToLowRPM = 500;
    private static final double kWheelRadius = 0.0636;

    private final double driveHighGearRatio;
    private final double driveLowGearRatio;
    private final double azimuthRatio;
    private final int moduleNumber;

    private final TalonFX drive;
    private final TalonFX azimuthTalon;
    private Slot0Configs azimuthPIDConfigs = new Slot0Configs();
    private CANSparkMax azimuthSpark;
    private SparkAbsoluteEncoder azimuthEncoder;
    private SparkPIDController azimuthPidController;
    private SparkLimitSwitch azimuthForwardLimit;
    private SparkLimitSwitch azimuthReverseLimit;
    public final DoubleSolenoid shifter;

    public ShiftedStates shiftedState = ShiftedStates.LOW;    

    private boolean azimuthSparkActive;

    // Used for update outputs, 0 indicates prior loop output
    private boolean unlockAzimuth0 = false;
    private boolean shifterOutput0 = false;
    private boolean unlockDrive0 = false;
    Timer shiftThreshold = new Timer();
    Timer robotDisabled = new Timer();

    private double driveSpeed = 0;
    private double azimuthAngle = 0;

    /**
     * Creates a new swerve module object
     * 
     * @param driveHighGearRatio The gear ratio between the drive motor and the
     *                           wheel in high gear
     * @param driveLowGearRatio  The gear ratio between the drive motor and the
     *                           wheel in low gear
     * @param azimuthRatio       The gear ratio between the azimuth motor and the
     *                           rotation
     *                           of the wheel
     * @param moduleNumber       The number of the module starting at 0 in the top
     *                           right
     *                           and increasing in a ccw circle
     * @param driveConfig        Configurations for the drive motor, use
     *                           SwerveUtils.swerveModuleDriveConfigs()
     * @param azimuthConfig      Configurations for the azimuth motor, use
     *                           SwerveUtils.swerveModuleAzimuthConfigs()
     * 
     */
    public SwerveModule(double driveHighGearRatio, double driveLowGearRatio, double azimuthRatio, int moduleNumber,
            TalonFXConfiguration driveConfig, TalonFXConfiguration azimuthConfig) {
        if (moduleNumber < 3) {
            shifter = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 2 - moduleNumber, 13 + moduleNumber);
        } else {
            shifter = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 3, 12);
        }
        shifter.set(Value.kForward);

        this.driveHighGearRatio = driveHighGearRatio;
        this.driveLowGearRatio = driveLowGearRatio;
        this.azimuthRatio = azimuthRatio;
        this.moduleNumber = moduleNumber;

        this.drive = new TalonFX(10 + moduleNumber, "canivore");
        this.azimuthTalon = new TalonFX(20 + moduleNumber, "canivore");

        azimuthSparkActive = false;

        try {
            this.azimuthTalon.get();
        } catch (Throwable err) {
            this.azimuthSpark = new CANSparkMax(20 + moduleNumber, MotorType.kBrushless);
            this.azimuthSpark.restoreFactoryDefaults();
            azimuthPidController = azimuthSpark.getPIDController();
            this.azimuthEncoder = azimuthSpark.getAbsoluteEncoder();
            azimuthSparkActive = true;
            this.azimuthForwardLimit = azimuthSpark.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
            this.azimuthReverseLimit = azimuthSpark.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        }

        // Apply configs and set PIDs
        if (!azimuthSparkActive) {
            this.azimuthTalon.getConfigurator().apply(azimuthConfig);
            azimuthPIDConfigs.kP = 0;
            azimuthPIDConfigs.kI = 0;
            azimuthPIDConfigs.kD = 0;
            this.azimuthTalon.getConfigurator().apply(azimuthPIDConfigs);
            this.azimuthTalon.setPosition(0.0);
        }

        this.drive.setPosition(0.0);
        this.drive.getConfigurator().apply(driveConfig);
    }



    public SwerveModuleState updateSensors() {
        if (azimuthSparkActive) {
            azimuthAngle = azimuthEncoder.getPosition() / azimuthRatio * 2 * Math.PI;
            if (azimuthForwardLimit.isPressed() && azimuthReverseLimit.isPressed()) {
                shiftedState = ShiftedStates.LOW;
                System.out.println("Error: High and low sensor are triggered at the same time on module " + moduleNumber);
            } else if (azimuthForwardLimit.isPressed()) {
                shiftedState = ShiftedStates.HIGH;
            } else if (azimuthReverseLimit.isPressed()) {
                shiftedState = ShiftedStates.LOW;
            } else {
                shiftedState = ShiftedStates.TRANS;
            }
        } else {

            boolean forwardLimit = azimuthTalon.getForwardLimit().getValue().equals(ForwardLimitValue.ClosedToGround);
            boolean reverseLimit = azimuthTalon.getReverseLimit().getValue().equals(ReverseLimitValue.ClosedToGround);
            if (forwardLimit && reverseLimit) {
                shiftedState = ShiftedStates.LOW;
                System.out.println("Error: High and low sensor are triggered at the same time on module " + moduleNumber);
            } else if (forwardLimit) {
                shiftedState = ShiftedStates.HIGH;
            } else if (reverseLimit) {
                shiftedState = ShiftedStates.LOW;
            } else {
                shiftedState = ShiftedStates.TRANS;
            }
            azimuthAngle = azimuthTalon.getRotorPosition().getValueAsDouble() / azimuthRatio * 2 * Math.PI;
        }

        driveSpeed = drive.getVelocity().getValueAsDouble() / ((shiftedState.equals(ShiftedStates.HIGH)) ? driveHighGearRatio : driveLowGearRatio) * Math.PI * Drivetrain.actualWheelDiameter;

        return new SwerveModuleState(driveSpeed, new Rotation2d(azimuthAngle));
    }

    /**
     * Returns the position of the drive and azimuth motors
     * 
     * @return A SwerveModulePosition object
     */
    public SwerveModulePosition getPosition() {
        Rotation2d rotation;
        // Decide between using spark and talon
        if (azimuthSparkActive) {
            rotation = new Rotation2d(azimuthEncoder.getPosition() / azimuthRatio * 2 * Math.PI);
        } else {
            rotation = new Rotation2d(
                    (azimuthTalon.getRotorPosition().getValueAsDouble() / azimuthRatio) * 2 * Math.PI);
        }
        return new SwerveModulePosition(
                (drive.getRotorPosition().getValueAsDouble()
                        / ((shifter.get() == Value.kForward) ? driveHighGearRatio : driveLowGearRatio))
                        * (2 * Math.PI * kWheelRadius),
                rotation);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState();
    }

    /**
     * Gets any faults from the drive and azimuth motors
     * 
     * @return A boolean array with the structure:
     *         <ul>
     *         <li>drive fault,
     *         <li>azimuth fault
     */
    public boolean[] getSwerveFaults() {
        // Decide between using spark and talon
        boolean azimuthFault;
        if (azimuthSparkActive) {
            azimuthFault = (short) 0 != azimuthSpark.getFaults();
        } else {
            azimuthFault = 0 != azimuthTalon.getFaultField().getValueAsDouble();
        }
        boolean driveFault = drive.getFaultField().getValueAsDouble() != 0;
        return new boolean[] { driveFault, azimuthFault };
    }

    /**
     * Sends modulestate outputs to the drive, azimuth, and shifter
     * 
     * @param moduleState
     * @param isAutonomous
     */
    public void updateOutputs(SwerveModuleState moduleState, boolean isAutonomous, boolean fLow,
            boolean moduleFailure, boolean homeWheels) {
        // Decide shifter output
        if (fLow) {
            shifterOutput0 = false;
        } else {
            if (isAutonomous) {
                shifterOutput0 = true;
            } else {
                if (shifterOutput0) {
                    // Currently commanded to high gear
                    if (Math.abs(drive.getVelocity().getValueAsDouble()) > shiftToLowRPM) {
                        shiftThreshold.reset();
                    }
                    shifterOutput0 = shiftThreshold.getTimeMillis() < 150;
                } else {
                    // Currently commanded to low gear
                    if (Math.abs(drive.getVelocity().getValueAsDouble()) < shiftToHighRPM) {
                        shiftThreshold.reset();
                    }
                    shifterOutput0 = shiftThreshold.getTimeMillis() > 150;
                }
            }
        }
        // Output to shifter
        ActuatorInterlocks.TAI_Solenoids(shifter, "Swerve_" + ((Integer) moduleNumber).toString() + "_Shifter_(b)",
                fLow);

        // Output to azimuth
        moduleState = SwerveModuleState.optimize(moduleState, new Rotation2d(azimuthAngle));

        double normalAzimuthOutput = Units.rotationsToRadians(((homeWheels) ? 0 : moduleState.angle.getRotations()));

        // Wrapping the angle to allow for "continuous input"
        double minDistance = MathUtil.angleModulus(normalAzimuthOutput - azimuthAngle);
        normalAzimuthOutput = Units.radiansToRotations(azimuthAngle + minDistance) * azimuthRatio;

        if (azimuthSparkActive) {
            ActuatorInterlocks.TAI_SparkMAX_Position(azimuthSpark, azimuthPidController,
                    "Azimuth_" + ((Integer) moduleNumber).toString() + "_(p)",
                    normalAzimuthOutput, 0.0);
        } else {
            /* PID tuning code */
            azimuthPIDConfigs.kP = Dashboard.freeTuningkP.get();
            azimuthPIDConfigs.kI = Dashboard.freeTuningkI.get();
            azimuthPIDConfigs.kD = Dashboard.freeTuningkD.get();
            this.azimuthTalon.getConfigurator().apply(azimuthPIDConfigs);

            ActuatorInterlocks.TAI_TalonFX_Position(azimuthTalon,
                    "Azimuth_" + ((Integer) moduleNumber).toString() + "_(p)",
                    normalAzimuthOutput, 0.0);
        }

        // Decide whether to put azimuth in coast mode
        boolean unlockAzimuth = Dashboard.unlockAzimuth.get();
        if ((unlockAzimuth != unlockAzimuth0) || moduleFailure) {
            if (unlockAzimuth || moduleFailure) {
                if (azimuthSparkActive) {
                    azimuthSpark.setIdleMode(IdleMode.kCoast);
                } else {
                    azimuthTalon.setNeutralMode(NeutralModeValue.Coast);
                }
            } else {
                if (azimuthSparkActive) {
                    azimuthSpark.setIdleMode(IdleMode.kBrake);
                } else {
                    azimuthTalon.setNeutralMode(NeutralModeValue.Brake);
                }
            }
        }
        unlockAzimuth0 = unlockAzimuth;

        

        // Output drive
        ActuatorInterlocks.TAI_TalonFX_Power(drive, "Drive_" + ((Integer) moduleNumber).toString() + "_(p)",
                moduleState.speedMetersPerSecond / Drivetrain.maxGroundSpeed);

        // unlock drive motor if robot is disabled for more than 7 seconds or module
        // fails
        if (DriverStation.isEnabled()) {
            robotDisabled.reset();
        }

        if (((robotDisabled.getTimeMillis() > 7000) != unlockDrive0) || moduleFailure) {
            if (moduleFailure || (robotDisabled.getTimeMillis() > 7000)) {
                drive.setNeutralMode(NeutralModeValue.Coast);
            } else {
                drive.setNeutralMode(NeutralModeValue.Brake);
            }
        }
        unlockDrive0 = robotDisabled.getTimeMillis() > 7000;
    }

    /**
     * Temperature of drive motor
     * 
     * <ul>
     * <li><b>Minimum Value:</b> 0.0
     * <li><b>Maximum Value:</b> 255.0
     * <li><b>Default Value:</b> 0
     * <li><b>Units:</b> ℃
     * </ul>
     * 
     * @return Double temperature in degrees Celcius
     */
    public double getTemp() {
        return drive.getDeviceTemp().getValueAsDouble();
    }
}
