package frc.robot.Utility;

import frc.robot.Dashboard;

public class ActuatorInterlocks {
    private static String testingActuator;
    private static double testingPeriod;
    private static double testingValue;

    public static boolean isTesting() {
        testingActuator = Dashboard.testActuatorName.get();
        return !testingActuator.equals("No_Test");
    }

    public static double TAI_Motors(String actuatorName, double normalOutput){
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();

        if (testingActuator.equals("No_Test")) {
            return normalOutput;
        }
        else if (testingActuator.equals(actuatorName)) {
            if (testingPeriod == 0){
                return testingValue;
            } else {
                return testingValue * Math.sin((double)System.currentTimeMillis() * 0.001 * Math.PI / testingPeriod);
            }
        }
        else {
            return 0;
        }
    }

    public static boolean TAI_Solenoids(String actuatorName, boolean normalOutput){
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();

        if (testingActuator.equals("No_Test")) {
            return normalOutput;
        }
        else if (testingActuator.equals(actuatorName)) {
            if (testingValue == 1.0) {
                return true;
            } else {
                return false;
            }
        }
        else {
            return false;
        }
    }

}
