package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;

public class AutonomyMenu {

    // String for the object name, such as "color"
    String objName;

    // Options for that object we created, such as "Blue" / "Red"
    ArrayList options;

    // Number of max objects in the menu
    int maxObjects;

    // current number of object we are in from the list
    int currentObjectNum = 1;

    // current number of item we are in from the object
    int currentItemNum = 1;

    // current number of option we are in from the object
    int currentOptionNum;

    // opmode members we need to set in the opmode
    Telemetry telemetry;
    Gamepad gamepad;

    // creating the files for saving.
    File objectFile;
    ArrayList<File> optionsFile;

    // Constructor for the class.
    public AutonomyMenu(Telemetry telemetry, int maxObjects, Gamepad gamepad) {
        this.telemetry = telemetry;
        this.maxObjects = maxObjects;
        this.gamepad = gamepad;
    }

    // Constructor for the class.
    public void newObject(String objName, ArrayList options) {
        this.objName = objName;
        this.options = options;
    }

    // declaring the files we need in order to save our objects and options to be read on autonomous code.
    public void createFiles() throws IOException {
        File filesOptions = null;
        objectFile = new File("Objects.txt");

        for(int i = 1; i <= options.size(); i++){
            filesOptions = new File("Options" + i + ".txt");
            optionsFile.add(filesOptions);
        }
    }

    // getter and setter for each class member

    public String getObjName() {
        return objName;
    }

    public void setObjName(String objName) {
        this.objName = objName;
    }

    public ArrayList getOptions() {
        return options;
    }

    public void setOptions(ArrayList options) {
        this.options = options;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int getMaxObjects() {
        return maxObjects;
    }

    public void setMaxObjects(int numItems) {
        this.maxObjects = numItems;
    }

    public void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public int getCurrentItemNum() {
        return currentItemNum;
    }

    public int getCurrentObjectNum() {
        return currentObjectNum;
    }

    // the fuction to display all the the object and there options on the telemetry //TODO: RUN  IN LOOP
    public void displayOnTelemetry(){

        // for each object we do this:
        for (int i = 1;  i <= maxObjects; i++){

            // scrolling with the gamepad up and down through the objects from the list, and checking
            // if the currentItemNum is at the max and you want to go up so jump to the first object,
            // and if the currentItemNum is at the min and you want to go down so jump to the max.
            if(gamepad.dpad_up){
                currentItemNum++;
            }else if(gamepad.dpad_up && currentItemNum > maxObjects){
                currentItemNum = 1;
            }else if(gamepad.dpad_down){
                currentItemNum--;
            }else if(gamepad.dpad_down && currentItemNum <= 0){
                currentItemNum = maxObjects;
            }

            // displaying the current object and its current option.
            //if(currentItemNum == i){
           //     telemetry.addData(" >> " + objName, options.get(currentOptionNum));
          //  }else{
                telemetry.addData(objName, options.get(currentObjectNum));
           // }

            // scrolling with the gamepad right and left through the options from the object, and checking
            // if the currentOptionNum is at the max and you want to go up so jump to the first option,
            // and if the currentOptionNum is at the min and you want to go down so jump to the max.
            if(gamepad.right_bumper){
                currentOptionNum++;
            }else if(gamepad.right_bumper && currentOptionNum > options.size()){
                currentOptionNum = 1;
            }else if(gamepad.left_bumper){
                currentOptionNum--;
            }else if(gamepad.left_bumper && currentOptionNum <= 0){
                currentOptionNum = options.size();
            }

        }

        telemetry.update();
    }

    // we save each object by writing the object name to 1 file, and for each option of the object we write to another file.
    public void saveObject(String objName){
        ReadWriteFile.writeFile(objectFile, objName + ",");

        for(int i = 1; i <= options.size(); i++){
            ReadWriteFile.writeFile(optionsFile.get(i), options.get(i) + ",");
        }
    }

    // we get the names of the objects from the saved files, and we return them as an ArrayList
    public ArrayList readObjectsNames() {
        ArrayList objectArray = new ArrayList();
        String lineSpliter = ",";

        for(int i = 1; i <= maxObjects; i++){
            String lines[] = ReadWriteFile.readFile(objectFile).split(lineSpliter);
            objectArray.add(lines[i]);
        }

        return objectArray;
    }

    // we get the name of the object we want to read the options from it
    // and we go through all of the object names using readObjectsNames(), and we
    // check want number of object is it from the list of names, and we get
    // all of its options and add it an ArrayList and return it
    public ArrayList readObjectOptions(String objName) throws IOException {
        ArrayList optionArray = new ArrayList();

        for(int i = 1; i <= readObjectsNames().size(); i++){
            if(readObjectsNames().get(i) == objName){
                for(int j = 1; j <= options.size(); j++){
                    optionArray.add(ReadWriteFile.readFile(optionsFile.get(i)));
                }

            }
        }

        return optionArray;
    }

}
