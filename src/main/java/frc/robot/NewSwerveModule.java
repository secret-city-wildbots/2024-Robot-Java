package frc.robot;

import frc.robot.Utility.ActuatorInterlocks;
import frc.robot.Utility.ClassHelpers.Timer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class NewSwerveModule {
    private final double shiftToHighRPM = 4000;
    private final double shiftToLowRPM = 500;
    private static final double kWheelRadius = 0.0636;

    private final double driveRatio;
    private final double azimuthRatio;
    private final int moduleNumber;

    private final TalonFX drive;
    private final TalonFX azimuthTalon;
    private CANSparkMax azimuthSpark;
    private SparkAbsoluteEncoder azimuthEncoder;
    private SparkPIDController azimuthPidController;
    private final DoubleSolenoid shifter;

    private boolean azimuthSparkActive;



    /**
     * Creates a new swerve module object
     * 
     * @param driveRatio The gear ratio between the drive motor and the wheel
     * @param azimuthRatio The gear ratio between the azimuth motor and the rotation of the wheel
     * @param moduleNumber The number of the module starting at 0 in the top right and increasing in a ccw circle
     * @param configs Configurations for the drive motor, use SwerveUtils.swerveModuleConfigs()
     * 
     */
    public NewSwerveModule(double driveRatio, double azimuthRatio, int moduleNumber, TalonFXConfiguration configs) {
        if (moduleNumber<3) {
            shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2-moduleNumber, 13+moduleNumber);
        } else {
            shifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, 3, 12);
        }
        shifter.set(Value.kForward);
        
        this.driveRatio = driveRatio;
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
        }

        this.drive.getConfigurator().apply(configs);
    }



    /**
     * Returns the position of the drive and azimuth motors
     * 
     * @return A SwerveModulePosition object
     */
    public SwerveModulePosition getPosition() {
        Rotation2d rotation;
        //Decide between using spark and talon
        if (azimuthSparkActive) {
            rotation = new Rotation2d(azimuthEncoder.getPosition() / azimuthRatio * 2 * Math.PI);
        } else {
            rotation = new Rotation2d((azimuthTalon.getRotorPosition().getValueAsDouble() / azimuthRatio) * 2 * Math.PI);
        }
        return new SwerveModulePosition(
        (drive.getRotorPosition().getValueAsDouble() / driveRatio) * (2 * Math.PI * kWheelRadius),
        rotation);
    }


    /**
     * Gets any faults from the drive and azimuth motors
     * 
     * @return A boolean array with the structure:
     *   <ul>
     *      <li> drive fault,
     *      <li> azimuth fault
     */
    public boolean[] getSwerveFaults() {
        // Decide between using spark and talon
        boolean azimuthFault;
        if (azimuthSparkActive) {
            azimuthFault = (short)0 != azimuthSpark.getFaults();
        } else {
            azimuthFault = 0 != azimuthTalon.getFaultField().getValueAsDouble();
        }
        boolean driveFault = drive.getFaultField().getValueAsDouble() != 0;
        return new boolean[]{driveFault, azimuthFault};
    }


    // 0 indicates prior loop output
    private boolean shifterOutput0 = false;
    Timer shiftThreshold = new Timer();
    /**
     * 
     * @param moduleState
     * @param isAutonomous
     */
    public void updateOutputs(SwerveModuleState moduleState, boolean isAutonomous, boolean fLow) {
        if (fLow) {
            shifterOutput0 = false;
        } else {
            if (isAutonomous) {
                shifterOutput0 = true;
            } else {
                if (shifterOutput0) {
                    // Currently commanded to high gear
                    if (Math.abs(drive.getVelocity().getValueAsDouble())>shiftToLowRPM) {
                        shiftThreshold.reset();
                    }
                    shifterOutput0 = shiftThreshold.getTimeMillis()<150;
                } else {
                    // Currently commanded to low gear
                    if (Math.abs(drive.getVelocity().getValueAsDouble())<shiftToHighRPM) {
                        shiftThreshold.reset();
                    }
                    shifterOutput0 = shiftThreshold.getTimeMillis()>150;
                }
            }
        }
    
        if (ActuatorInterlocks.TAI_Solenoids("Swerve_" +((Integer)moduleNumber).toString() + "_Shifter_(b)", fLow)) {
            shifter.set(Value.kForward);
        } else {
            shifter.set(Value.kReverse);
        }


        double azimuthOutput = ActuatorInterlocks.TAI_Motors("Azimuth_" + ((Integer)moduleNumber).toString() + "_(p)", moduleState.angle.getRotations() * azimuthRatio);
        double driveOutput = ActuatorInterlocks.TAI_Motors("Drive_" + ((Integer)moduleNumber).toString() + "_(p)", moduleState.speedMetersPerSecond);

        if (azimuthSparkActive) {
            azimuthPidController.setReference(azimuthOutput, CANSparkBase.ControlType.kPosition);
        } else {
            PositionDutyCycle controlRequest = new PositionDutyCycle(azimuthOutput);
            azimuthTalon.setControl(controlRequest);
        }

        drive.set(driveOutput);
    }



    /**
     * Temperature of drive motor
     * 
     * <ul>
     *   <li> <b>Minimum Value:</b> 0.0
     *   <li> <b>Maximum Value:</b> 255.0
     *   <li> <b>Default Value:</b> 0
     *   <li> <b>Units:</b> ℃
     * </ul>
     * 
     * @return Double temperature in degrees Celcius
     */
    public double getTemp() {
        return drive.getDeviceTemp().getValueAsDouble();
    }
}
