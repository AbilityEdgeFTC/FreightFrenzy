package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.util.Controller;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;

@TeleOp(group="Tests")
public class MenuOpModeTest extends LinearOpMode
{

    int maxTasks = 0, maxOptions = 0, taskNum = 0;
    boolean flag = false;
    String taskName = "";

    // the list of the tasks, and their options(list in list)
    ArrayList<ArrayList<String>> tasks = new ArrayList<ArrayList<String>>();
    // the tasks' names
    ArrayList<String> tasksName = new ArrayList<String>();

    // list for each task
    ArrayList<String> colorTask = new ArrayList<String>();
    ArrayList<String> parkTask = new ArrayList<String>();
    ArrayList<String> carouselTask = new ArrayList<String>();
    ArrayList<Integer> order = new ArrayList<Integer>();

    // list of the final options the player chose
    ArrayList<String> finalOptions = new ArrayList<String>();

    ArrayList<Integer> currentOption = new ArrayList<Integer>();

    enum tasksEnum
    {
        color,
        park,
        carousel
    }

    @Override
    public void runOpMode()
    {
        Controller controller = new Controller(gamepad1);

//        File myFile = new File("options.txt");
//
//        try {
//            if(myFile.delete() && !myFile.createNewFile()){
//                try {
//                    myFile.createNewFile();
//                } catch (IOException e) {
//                    e.printStackTrace();
//                }
//            }
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

        // adding the options to color task
        colorTask.add("Blue");
        colorTask.add("Red");

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
        tasks.add(parkTask);
        tasks.add(carouselTask);

        // adding the names of the tasks to the list of tasks' names.
        tasksName.add("color");
        tasksName.add("park");
        tasksName.add("carousel");


        for(int i = 0; i < tasks.size(); i++)
        {
            order.add(i, i);
            currentOption.add(i, 0);
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
                finalOptions.add(taskNum, tasks.get(taskNum).get(currentOption.get(taskNum)));
                taskNum--;
            }
            if(controller.dpadDownOnce() && taskNum < maxTasks)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(currentOption.get(taskNum)));
                taskNum++;
            }

            if(controller.dpadLeftOnce() && currentOption.get(taskNum) > 0)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(currentOption.get(taskNum)));
                currentOption.set(taskNum, currentOption.get(taskNum)-1);
            }
            if(controller.dpadRightOnce() && currentOption.get(taskNum) < maxOptions)
            {
                finalOptions.add(taskNum, tasks.get(taskNum).get(currentOption.get(taskNum)));
                currentOption.set(taskNum, currentOption.get(taskNum)+1);
            }

            telemetry.addLine("(" + (taskNum+1) + ") " + taskName + ": " + tasks.get(taskNum).get(currentOption.get(taskNum)));

            if(gamepad1.a)
            {
                for(int i = 0; i < finalOptions.size(); i++)
                {
                    switch(i)
                    {
                        case 0:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("color.txt"), finalOptions.get(i));
                            break;
                        case 1:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("park.txt"), finalOptions.get(i));
                            break;
                        case 2:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("carousel.txt"), finalOptions.get(i));
                            break;
                    }
                }
                flag = true;
            }

            telemetry.update();

        }



    }

    public ArrayList<Pose2d> listOfPose()
    {
        ArrayList<Pose2d> pose2DS = new ArrayList<>();
        boolean blue = false, red = false;

        for(int i = 0; i <= finalOptions.size(); i++)
        {
            if(colorTask.get(0).equals(finalOptions.get(i)))
            {
                blue = true;
                red = false;
            }
            else if(colorTask.get(1).equals(finalOptions.get(i)))
            {
                blue = false;
                red = true;
            }

            if(parkTask.get(0).equals(finalOptions.get(i)))
            {
                if(blue)
                {
                    pose2DS.add(i, new Pose2d(1,1, 0));
                }
                else
                {
                    pose2DS.add(i, new Pose2d(-1,-1, 0));
                }
            }
            else if(parkTask.get(1).equals(finalOptions.get(i)))
            {
                if(blue)
                {
                    pose2DS.add(i, new Pose2d(2,2, 0));
                }
                else
                {
                    pose2DS.add(i, new Pose2d(-2,-2, 0));
                }
            }
            else if(parkTask.get(2).equals(finalOptions.get(i)))
            {
                if(blue)
                {
                    pose2DS.add(i, new Pose2d(3,3, 0));
                }
                else
                {
                    pose2DS.add(i, new Pose2d(-3,-3, 0));
                }
            }
            else if(parkTask.get(3).equals(finalOptions.get(i)))
            {
                if(blue)
                {
                    pose2DS.add(i, new Pose2d(4,4, 0));
                }
                else
                {
                    pose2DS.add(i, new Pose2d(-4,-4, 0));
                }
            }

            if(carouselTask.get(0).equals(finalOptions.get(i)))
            {
                if(blue)
                {
                    pose2DS.add(i, new Pose2d(1,1, 0));
                }
                else
                {
                    pose2DS.add(i, new Pose2d(-1,-1, 0));
                }
            }
            else if(carouselTask.get(1).equals(finalOptions.get(i)))
            {
                if(blue)
                {
                    pose2DS.add(i, new Pose2d(2,2, 0));
                }
                else
                {
                    pose2DS.add(i, new Pose2d(-2,-2, 0));
                }
            }
        }

        return pose2DS;
    }
}