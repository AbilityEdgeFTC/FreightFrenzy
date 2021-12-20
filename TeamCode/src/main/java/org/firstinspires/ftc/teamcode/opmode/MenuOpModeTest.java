package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.util.Controller;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

@TeleOp(group="Tests")
public class MenuOpModeTest extends LinearOpMode
{

    int maxTasks = 0, maxOptions = 0, taskNum = 0, optionNum = 0;
    boolean flag = false;
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
    ArrayList<Integer> order = new ArrayList<Integer>();

    // list of the final options the player chose
    ArrayList<String> finalOptions = new ArrayList<String>();

    @Override
    public void runOpMode()
    {
        Controller controller = new Controller(gamepad1);

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

        for(int i = 0; i < tasks.size(); i++)
        {
            order.add(i);
        }



        // max tasks is the size of the list of tasks.
        maxTasks = tasks.size()-1;

        // wait for the game to start (driver presses PLAY)
        waitForStart();

        // run while player didn't press stop
        while (opModeIsActive() && !flag)
        {

            controller.update();
            taskName = tasksName.get(taskNum);
            maxOptions = tasks.get(taskNum).size()-1;

            if(controller.dpadUpOnce() && taskNum > 0)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                taskNum--;
            }
            if(controller.dpadDownOnce() && taskNum < maxTasks)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                taskNum++;
            }

            if(controller.dpadLeftOnce() && optionNum > 0)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                optionNum--;
            }
            if(controller.dpadRightOnce() && optionNum < maxOptions)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(optionNum));
                optionNum++;
            }

            telemetry.addLine("(" + (taskNum+1) + ") " + taskName + ": " + tasks.get(taskNum).get(optionNum));

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
                flag = true;
            }

            telemetry.update();
        }



    }
}