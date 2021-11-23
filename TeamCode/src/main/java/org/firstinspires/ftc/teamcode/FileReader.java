package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class FileReader {
    File myFile = new File("RRconfig.txt");
    String[] lines = {""};
    String[] headers = {""};
    String[] letters = {""};
    int numOfLines = 0;

    public void readFile(Telemetry telemetry){
        try{
            Scanner myReader = new Scanner(myFile);
            while(myReader.hasNextLine()){
                numOfLines++;
            }
            for(int i = 1; i <= numOfLines; i++){
                lines[i] = myReader.nextLine();
                if(lines[i].contains(":")){
                    headers[i] = String.valueOf(lines[i].split("[:]"));
                }
                for(int j = 0; j < headers[i].length(); j++){
                    letters[j] = String.valueOf(headers[i].charAt(j));
                    telemetry.addLine(letters[j]);
                    telemetry.update();
                }
                //lines[i].split("[-]");
            }
            myReader.close();
        }catch (FileNotFoundException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }


    }
}