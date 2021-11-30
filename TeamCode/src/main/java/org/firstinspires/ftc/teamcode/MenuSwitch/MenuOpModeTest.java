/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.MenuSwitch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutonomyMenu;

import java.util.ArrayList;

@TeleOp(name="Autonomy Menu Test" , group="Tests")
public class MenuOpModeTest extends LinearOpMode {

    // TODO: HERE WE DECLARE THE NAMES OF THE OBJECTS AND THE ARRAYS


    // Options for that object we created, such as "Blue" / "Red"

    //the string name of the option specific object now
    String currentObject = "";
    //the string name of the option now
    String currentOption = "";

    // Number of max options in the menu
    int maxOptions ;


    int currentObjectNum = 1;

    int currentOptionNum = 1;





    // opmode members we need to set in the opmode
    Telemetry telemetry;
    Gamepad gamepad;



    //options list
    ArrayList<ArrayList<String>> options = new ArrayList<ArrayList<String>>();

    //lisy for each option
    ArrayList<String> colorOPT = new ArrayList<String>();
    ArrayList<String> VisionOPT = new ArrayList<String>();
    ArrayList<String> ParkOPT = new ArrayList<String>();
    ArrayList<String> CarouselOPT = new ArrayList<String>();
    //the currnent option list
    ArrayList<String> CurentObject = new ArrayList<String>();
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // TODO: HERE WE CREATE THE OBJECT and FILES using the createFiles function FROM THE CLASS AutonomyMenu.A

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


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        //UP AND DOWN
        while (opModeIsActive()) {
            if(gamepad.dpad_up){
                currentOptionNum--;
                CurentObject = options.get(currentOptionNum);
                currentOption = String.valueOf(options.get(currentOptionNum));

            }else if(gamepad.dpad_down){
                currentOptionNum++;
                CurentObject = options.get(currentOptionNum);
                currentOption = String.valueOf(options.get(currentOptionNum));

            }else if(gamepad.dpad_up && currentOptionNum<= 0){
                currentOptionNum = maxOptions;
                CurentObject = options.get(currentOptionNum);
                currentOption = String.valueOf(options.get(currentOptionNum));

            }else if(gamepad.dpad_down && currentOptionNum >maxOptions){
                currentOptionNum = 1;
                CurentObject = options.get(currentOptionNum);
                currentOption = String.valueOf(options.get(currentOptionNum));

            }

            //LEFT AND RIGHT
            if(gamepad.right_bumper){
                currentObjectNum++;
                telemetry.addData(currentOptionNum+": ",options.get(currentOptionNum));
            }else if(gamepad.right_bumper && currentOptionNum >= options.size()){
                currentObjectNum = 1;
            }else if(gamepad.left_bumper){
                currentObjectNum--;
            }else if(gamepad.left_bumper && currentOptionNum <= 0){
                currentObjectNum = CurentObject.size();
            }
            telemetry.addLine(currentOption+": ");
            telemetry.update();
                //TODO: HERE WE CAN USE THE displayOnTelemetry function.
            
        }

        // TODO: HERE WE SAVE EACH AND EACH OBJECT

    }
}
