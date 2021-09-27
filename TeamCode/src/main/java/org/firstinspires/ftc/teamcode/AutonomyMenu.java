package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    FileWriter objectFilesWrite;
    ArrayList<FileWriter> optionFilesWrite;
    FileReader objectFilesRead;
    ArrayList<FileReader> optionFilesRead;

    // declaring the files we need in order to save our objects and options to be read on autonomous code.
    public void createFiles() throws IOException {
        objectFilesWrite = new FileWriter("Objects.txt");

        for(int i = 1; i <= options.size(); i++){
            optionFilesWrite = new ArrayList<FileWriter>((Collection<? extends FileWriter>) new FileWriter("Objects" + i + ".txt"));
        }
    }


    // Constructor for the class.
    public AutonomyMenu(String objName, ArrayList options, Telemetry telemetry, int maxObjects, Gamepad gamepad) {
        this.objName = objName;
        this.options = options;
        this.telemetry = telemetry;
        this.maxObjects = maxObjects;
        this.gamepad = gamepad;
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
            if(currentItemNum == i){
                telemetry.addData(" >> " + objName, options.get(currentOptionNum));
            }else{
                telemetry.addData(objName, options.get(currentObjectNum));
            }

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
    public void saveObject(String objName, ArrayList options) throws IOException {
        objectFilesWrite.write(objName + "\\n");

        for(int i = 1; i <= options.size(); i++){
            optionFilesWrite.get(i).write(options.get(i) + "\\n");
        }
    }

    //TODO:
    public void readObject(String objName){

    }

}
