package frc.robot.Utility;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class FileHelpers {



    /**
     * Reads each line of a file as plain text
     * @param path The file path location to find the file at. For the robot this is usually /home/lvuser/...
     * @return A string containing each character in the file
     */
    public static String readFile(String path) {
        String contents = "";
        try {
            File myObj = new File(path);
            Scanner myReader = new Scanner(myObj);
            while (myReader.hasNextLine()) {
                String data = myReader.nextLine();
                contents += data;
            }
            myReader.close();
        } catch (FileNotFoundException e) {
            System.out.println("File not found!");
            e.printStackTrace();
        }
        return contents;
    }



    /**
     * Reads in a .csv file at the given file path, splits the file at each new line and returns a 2d double array where 
     * the rows are the new lines and columns are the values
     * @param path The file path location to find the file at. For the robot this is usually /home/lvuse/...
     * @return A double array containing the values in the csv
     */
    public static double[][] parseCSV(String path) {
        // Read in the file and split at each new line
        String file = readFile(path);
        String[] fileSetpoints = file.split("\r\n");
        fileSetpoints[0] = "";

        // Parse plain text strings into doubles and places them in the output array
        double[][] outputArray = new double[fileSetpoints.length-1][fileSetpoints.length-1];
        int i = 0;
        for (String x: fileSetpoints) {
            if (!x.equals("")) {
                outputArray[i][0] = Double.parseDouble(x.split(",")[0]);
                outputArray[i][1] = Double.parseDouble(x.split(",")[1]);
                i += 1;
            }
        }

        return outputArray;
    }
}
