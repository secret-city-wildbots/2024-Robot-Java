package frc.robot.Utility;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class FileHelpers {
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

    public static double[][] parseCSV(String path) {
        String file = readFile(path);
        String[] fileSetpoints = file.split("\r\n");
        fileSetpoints[0] = "";
        double[][] outputArray = new double[fileSetpoints.length-1][fileSetpoints.length-1];
        int i = 0;
        for (String x: fileSetpoints) {
            if (!x.equals("")) {
                outputArray[i][0] = Double.parseDouble(x.split(",")[0]);
                outputArray[i][1] = Double.parseDouble(x.split(",")[1]);
                i += 1;
            }
        }
        return outputArray; // Completely wrong but this is temp
    }
}
