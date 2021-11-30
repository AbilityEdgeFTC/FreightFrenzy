package org.firstinspires.ftc.teamcode.MenuSwitch;

import android.icu.text.CurrencyPluralInfo;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Menu {

    // String for the object name, such as "color"
    String objName;

    // Options for that object we created, such as "Blue" / "Red"
    String optionName;

    String curentObject = "";

    // Number of max objects in the menu
    int maxOptions = 0;

    // current number of object we are in from the list
    //int currentObjectNum = 1;

    // current number of item we are in from the object
    int currentItemNum = 1;

    // current number of option we are in from the object
    int currentOptionNum = 1;

    // opmode members we need to set in the opmode
    Telemetry telemetry;
    Gamepad gamepad;


    public void displayOnTelemetry(Telemetry telemetry){

        ArrayList<ArrayList<String>> options = new ArrayList<ArrayList<String>>();

        ArrayList<String> colorOPT = new ArrayList<String>();
        ArrayList<String> VisionOPT = new ArrayList<String>();
        ArrayList<String> ParkOPT = new ArrayList<String>();
        ArrayList<String> CarouselOPT = new ArrayList<String>();

        ArrayList<String> CurentObject = new ArrayList<String>();


        //Color Option ArrayList
        colorOPT.add("Blue");
        colorOPT.add("Red");
        //VisionType Option Array
        VisionOPT.add("TSE");
        VisionOPT.add("Duck");
        VisionOPT.add("No Vision");
        //Park Option ArrayList
        ParkOPT.add("Parked In Alliance Storage Unit");
        ParkOPT.add("Parked Completely In Alliance Storage Unit");
        ParkOPT.add("Parked In Warehouse");
        ParkOPT.add("Parked Completely In Warehouse");
        //Carousel Option ArrayList
        CarouselOPT.add("Off");
        CarouselOPT.add("On");


        options.add(colorOPT);
        options.add(VisionOPT);
        options.add(ParkOPT);
        options.add(CarouselOPT);

        maxOptions = options.size();

        // for each object we do this:
       // for (int i = 1;  i <= maxObjects; i++){

            // scrolling with the gamepad up and down through the objects from the list, and checking
            // if the currentItemNum is at the max and you want to go up so jump to the first object,
            // and if the currentItemNum is at the min and you want to go down so jump to the max.
            if(gamepad.dpad_up){
                currentOptionNum++;
                CurentObject = options.get(currentItemNum);
                curentObject = String.valueOf(options.get(currentItemNum));
                telemetry.addData(curentObject+": ", CurentObject);
            }else if(gamepad.dpad_down){
                currentItemNum--;
                CurentObject = options.get(currentItemNum);
                curentObject = String.valueOf(options.get(currentItemNum));
                telemetry.addData(curentObject+": ", CurentObject);
            }else if(gamepad.dpad_up && currentOptionNum> maxOptions){
                currentItemNum = 1;
                CurentObject = options.get(currentItemNum);
                curentObject = String.valueOf(options.get(currentItemNum));
                telemetry.addData(curentObject+": ", CurentObject);
            }else if(gamepad.dpad_down && currentItemNum <= 0){
                currentItemNum = maxOptions;
                CurentObject = options.get(currentItemNum);
                curentObject = String.valueOf(options.get(currentItemNum));
                telemetry.addData(curentObject+": ", CurentObject);
            }


            // displaying the current object and its current option.
            //if(currentItemNum == i){
            //     telemetry.addData(" >> " + objName, options.get(currentOptionNum));
            //  }else{
     //       telemetry.addData(options.get(currentOptionNum));
            // }

            // scrolling with the gamepad right and left through the options from the object, and checking
            // if the currentOptionNum is at the max and you want to go up so jump to the first option,
            // and if the currentOptionNum is at the min and you want to go down so jump to the max.
            if(gamepad.right_bumper){
                currentItemNum++;
                telemetry.addData(currentOptionNum+": ",options.get(currentOptionNum));
            }else if(gamepad.right_bumper && currentOptionNum >= options.size()){
                currentItemNum = 1;
            }else if(gamepad.left_bumper){
                currentItemNum--;
            }else if(gamepad.left_bumper && currentOptionNum <= 0){
                currentItemNum = CurentObject.size();
            }

        //}

        telemetry.update();
    }


}
