package frc.robot.Utility;

import frc.robot.Dashboard;

public class ActuatorInterlocks {
    private static String testingActuator;
    private static double testingPeriod;
    private static double testingValue;



    /**
     * Fetches whether or not an actuator is being tested from the Dashboard 
     * @return Whether the dashboard is testing an actuator
     */
    public static boolean isTesting() {
        testingActuator = Dashboard.testActuatorName.get();
        return !testingActuator.equals("No_Test");
    }



    /**
     * Actuator testing:
     * <ul>   
     * <li> If the Dashboard is not testing anything, return normal output
     * <li> If the Dashboard is testing this actuator:
     *  <ul>
     *  <li> Return the amplitude from the Dashboard as a power output if no period is present
     *  <li> Return a time based sinusoid with amplitude from the Dashboard as a power output if a period is given
     *  </ul>
     * <li> If the Dashboard is testing a different actuator, return 0
     * </ul>
     * 
     * This returns a double output for motor testing
     *      
     * @param actuatorName The name of the actuator matching the list in Robot.java
     * @param normalOutput The output that would go to the motor while not in testing mode
     * @return A motor output to implement actuator testing
     */
    public static double TAI_Motors(String actuatorName, double normalOutput){
        // Get Dashboard testing values
        testingActuator = Dashboard.testActuatorName.get();
        testingPeriod = Dashboard.testActuatorPeriod.get();
        testingValue = Dashboard.testActuatorValue.get();

        //If nothing is being tested, return the normal output. Otherwise, uses dashboard amplitude as double output or periodic sine output
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



    /**
     * Actuator testing:
     * <ul>   
     * <li> If the Dashboard is not testing anything, return normal output
     * <li> If the Dashboard is testing this actuator, return true or false if the Dashboard amplitude is 0 or 1 respectively
     * <li> If the Dashboard is testing a different actuator, return 0
     * </ul><br>
     * 
     * This returns a boolean output for a solenoid/piston
     * 
     * @param actuatorName
     * @param normalOutput
     * @return
     */
    public static boolean TAI_Solenoids(String actuatorName, boolean normalOutput){
        // Get dashboard test values
        testingActuator = Dashboard.testActuatorName.get();
        testingValue = Dashboard.testActuatorValue.get();

        // If nothing is being tested, return the normal output. Otherwise, uses dashboard amplitude as 1 or 0 for an output
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
