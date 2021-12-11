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

package org.firstinspires.ftc.teamcode.opmode.MenuSwitch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp(name="Autonomy Menu Test" , group="Tests")
public class MenuOpModeTest extends LinearOpMode
{

    int maxTasks = 0, maxOptions = 0, taskNum = 0, optionNum = 0;
    boolean flag = false, flag2 = false, flag3;
    String taskName = "";

    // the list of the tasks, and their options(list in list)
    ArrayList<ArrayList<String>> tasks = new ArrayList<ArrayList<String>>();
    // the tasks' names
    ArrayList<String> tasksName = new ArrayList<String>();

    // list for each task
    ArrayList<String> colorTask = new ArrayList<String>();
    ArrayList<String> VisionTask = new ArrayList<String>();
    ArrayList<String> parkTask = new ArrayList<String>();
    ArrayList<String> carouselTask = new ArrayList<String>();

    // list of the final options the player chose
    ArrayList<String> finalOptions = new ArrayList<String>();

    @Override
    public void runOpMode()
    {
        File myFile = new File("options.txt");

        try {
            if(myFile.delete() && !myFile.createNewFile()){
                try {
                    myFile.createNewFile();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        } catch (IOException e) {
            e.printStackTrace();
        }

        // adding the options to color task
        colorTask.add("Blue");
        colorTask.add("Red");

        // adding the options to vision task
        VisionTask.add("TSE");
        VisionTask.add("Duck");
        VisionTask.add("No Vision");

        // adding the options to park task
        parkTask.add("Not Completely In ASU");
        parkTask.add("Completely In ASU");
        parkTask.add("Not Completely In WH");
        parkTask.add("Completely In WH");

        // adding the options to carousel task
        carouselTask.add("Yes");
        carouselTask.add("No");

        // adding all the tasks to list of tasks
        tasks.add(colorTask);
        tasks.add(VisionTask);
        tasks.add(parkTask);
        tasks.add(carouselTask);

        // adding the names of the tasks to the list of tasks' names.
        tasksName.add("color");
        tasksName.add("vision");
        tasksName.add("park");
        tasksName.add("carousel");

        // max tasks is the size of the list of tasks.
        maxTasks = tasks.size();

        // wait for the game to start (driver presses PLAY)
        waitForStart();

        // run while player didn't press stop
        while (opModeIsActive() && !flag3)
        {

            // if player pressed dpad up, and flag(helps to run the if only once when pressed once) not true so:
            if(gamepad1.dpad_up && !flag)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                // we go down a task
                taskNum--;
                // we check if the current task num is <= 0, and if yes so:
                if(taskNum <= 0)
                {
                    // we jump to the last task in the list
                    taskNum = maxTasks;
                    // the max options will be the number of options in the current task number
                    maxOptions = tasks.get(taskNum).size();
                    // the name of the current task will be the index of taskNum from the list of tasks' name
                    taskName = tasksName.get(taskNum);
                }
                // flag true, to exist
                flag = true;
            }
            else if(gamepad1.dpad_down && !flag)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                taskNum++;
                if(taskNum > maxTasks)
                {
                    taskNum = 1;
                    maxOptions = tasks.get(taskNum).size();
                    taskName = tasksName.get(taskNum);
                }
                flag = true;
            }

            if(!gamepad1.dpad_down)
            {
                flag = false;
            }
            if(!gamepad1.dpad_up)
            {
                flag = false;
            }



            if(gamepad1.dpad_left && !flag2)
            {
                optionNum--;
                if(optionNum <= 0)
                {
                    optionNum = maxOptions;
                }
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                flag2 = true;
            }
            else if(gamepad1.dpad_right && !flag2)
            {
                optionNum++;
                if(optionNum > maxTasks)
                {
                    optionNum = 1;
                }
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                flag2 = true;
            }


            if(!gamepad1.dpad_down)
            {
                flag2 = false;
            }
            if(!gamepad1.dpad_up)
            {
                flag2 = false;
            }

            for(int i = 0; i < maxTasks; i++)
            {
                if(i == taskNum)
                {
                    telemetry.addData("-> ", taskName + "", tasks.get(taskNum).get(optionNum));
                }
                else
                {
                    telemetry.addData(taskName + "", tasks.get(taskNum).get(optionNum));
                }
            }

            if(gamepad1.a)
            {
                try {
                    FileWriter myWriter = new FileWriter(myFile);
                    for(int j = 0; j < finalOptions.size(); j++)
                    {
                        myWriter.write(finalOptions.get(j)+"\n");
                    }
                    myWriter.close();
                } catch (IOException e) {
                    telemetry.addLine("ERROR COULD NOT SAVE.");
                }
                flag3 = true;
            }

            telemetry.update();
        }



    }
}
