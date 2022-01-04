package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.Controller;

import java.lang.reflect.Array;
import java.util.ArrayList;

@TeleOp(group="Tests")
public class MenuOpModeTest extends LinearOpMode
{

    int maxTasks = 0, maxOptions = 0, taskNum = 0;
    boolean flag = false;
    String taskName = "";

    String[][] tasks = {};
    String[] tasksName = {};
    String[] colorTask = {};
    String[] parkTask = {};
    String[] carouselTask = {};
    String[] finalOptions = {};
    int[] currentOption = {};

    @Override
    public void runOpMode()
    {
        Controller controller = new Controller(gamepad1);

        // adding the options to color task
        colorTask[0] = "Blue";
        colorTask[1] = "Red";

        parkTask[0] = "Not Completely In ASU";
        parkTask[1] = "Completely In ASU";
        parkTask[2] = "Not Completely In WH";
        parkTask[3] = "Completely In WH";

        carouselTask[0] = "Yes";
        carouselTask[1] = "No";

        tasks[0] = colorTask;
        tasks[1] = parkTask;
        tasks[2] = carouselTask;

        // adding the names of the tasks to the list of tasks' names.
        tasksName[0] = "color";
        tasksName[1] = "park";
        tasksName[2] = "carousel";


        for(int i = 0; i < tasks.length; i++)
        {
            currentOption[i] = 0;
        }

        // max tasks is the size of the list of tasks.
        maxTasks = tasks.length-1;

        // wait for the game to start (driver presses PLAY)
        waitForStart();

        // run while player didn't press stop
        while (opModeIsActive() && !flag)
        {

            controller.update();
            taskName = tasksName[taskNum];
            maxOptions = tasks[taskNum].length-1;

            if(controller.dpadUpOnce() && taskNum > 0)
            {
                finalOptions[taskNum] =  tasks[taskNum][currentOption[taskNum]];
                taskNum--;
            }
            if(controller.dpadDownOnce() && taskNum < maxTasks)
            {
                finalOptions[taskNum] =  tasks[taskNum][currentOption[taskNum]];
                taskNum++;
            }

            if(controller.dpadLeftOnce() && currentOption[taskNum] > 0)
            {
                finalOptions[taskNum] = tasks[taskNum][currentOption[taskNum]];
                currentOption[taskNum] = currentOption[taskNum]-1;
            }
            if(controller.dpadRightOnce() && currentOption[taskNum] < maxOptions)
            {
                finalOptions[taskNum] = tasks[taskNum][currentOption[taskNum]];
                currentOption[taskNum] = currentOption[taskNum]+1;
            }

            telemetry.addLine("(" + (taskNum+1) + ") " + taskName + ": " + tasks[taskNum][currentOption[taskNum]]);

            if(gamepad1.a)
            {
                for(int i = 0; i < finalOptions.length; i++)
                {
                    switch(i)
                    {
                        case 0:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("color.txt"), finalOptions[i]);
                            break;
                        case 1:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("park.txt"), finalOptions[i]);
                            break;
                        case 2:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("carousel.txt"), finalOptions[i]);
                            break;
                    }
                }
                flag = true;
            }

            telemetry.update();

        }



    }
}