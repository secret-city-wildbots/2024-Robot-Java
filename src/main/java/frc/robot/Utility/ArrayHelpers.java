package frc.robot.Utility;

public class ArrayHelpers {
    public static double[] getColumn(double[][] array, int index){
        double[] column = new double[array[index].length];
        for(int i=0; i<column.length; i++){
           column[i] = array[i][index];
        }
        return column;
    }
}
