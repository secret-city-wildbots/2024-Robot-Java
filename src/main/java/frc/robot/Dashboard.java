package frc.robot;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;

public class Dashboard {
    @SuppressWarnings("unused")
    private NetworkTableInstance inst;
    @SuppressWarnings("unused")
    private NetworkTable table;

    // Array publishers
    public static StringArrayPublisher legalActuatorNames;
    public static StringArrayPublisher legalDrivers;
    public static DoubleArrayPublisher swerve0Details;
    public static DoubleArrayPublisher swerve1Details;
    public static DoubleArrayPublisher swerve2Details;
    public static DoubleArrayPublisher swerve3Details;

    // Subscribers
    public static StringSubscriber testActuatorName;
    public static DoubleSubscriber testActuatorValue;
    public static DoubleSubscriber testActuatorPeriod;
    public static DoubleSubscriber selectedDriver;
    public static BooleanSubscriber unlockAzimuth;



    /**
     * Creates an object for storing dashboard publishers and subscribers
     */
    public Dashboard(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("SmartDashboard");

        legalActuatorNames = table.getStringArrayTopic("Legal_Actuator_Names").publish();
        legalDrivers = table.getStringArrayTopic("Legal_Drivers").publish();
        swerve0Details = table.getDoubleArrayTopic("Swerve_0_Details").publish();
        swerve1Details = table.getDoubleArrayTopic("Swerve_1_Details").publish();
        swerve2Details = table.getDoubleArrayTopic("Swerve_2_Details").publish();
        swerve3Details = table.getDoubleArrayTopic("Swerve_3_Details").publish();

        testActuatorName = table.getStringTopic("Test_Actuator_Name").subscribe("");
        testActuatorValue = table.getDoubleTopic("Test_Actuator_Value").subscribe(0.0);
        testActuatorPeriod = table.getDoubleTopic("Test_Actuator_Period").subscribe(0.0);
        selectedDriver = table.getDoubleTopic("Selected_Driver").subscribe(0.0);
        unlockAzimuth = table.getBooleanTopic("Unlock_Azimuth").subscribe(false);
    }
}
