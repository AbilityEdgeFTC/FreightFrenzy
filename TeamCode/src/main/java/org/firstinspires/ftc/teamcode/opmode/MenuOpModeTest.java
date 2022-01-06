package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;

import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.carouselTask;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.colorTask;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.currentOption;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.finalOptions;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.parkTask;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.tasks;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.tasksName;
import static org.firstinspires.ftc.teamcode.robot.subsystems.valueStorage.taskName;

@TeleOp(group="Tests")
public class MenuOpModeTest extends LinearOpMode
{

    int maxTasks = 0, maxOptions = 0, taskNum = 0;
    boolean flag = false;

    @Override
    public void runOpMode()
    {
        cGamepad cGamepad = new cGamepad(gamepad1);
        initTasks();

        // orders of the tasks //TODO: CHANGE
        tasks[0] = colorTask;
        tasks[1] = carouselTask;
        tasks[2] = parkTask;

        // adding the names of the tasks to the list of tasks' names. //TODO: CHANGE
        tasksName[0] = "color";
        tasksName[1] = "carousel";
        tasksName[2] = "park";

        // set every current option by the number of tasks to 0
        for(int i = 0; i < tasks.length; i++)
        {
            currentOption[i] = 0;
        }

        // max tasks is the size of the list of tasks - 1.
        maxTasks = tasks.length-1;

        // wait for the game to start (driver presses PLAY)
        waitForStart();

        // run while player didn't press stop
        while (opModeIsActive() && !flag)
        {

            cGamepad.update();
            // set the cuurent task name to the one fro the tasksName array at the taskNum index
            taskName = tasksName[taskNum];
            // setting the max options to the number of options for current task - 1
            maxOptions = tasks[taskNum].length-1;

            if(cGamepad.dpadUpOnce() && taskNum > 0)
            {
                finalOptions[taskNum] =  tasks[taskNum][currentOption[taskNum]];
                taskNum--;
            }
            if(cGamepad.dpadDownOnce() && taskNum < maxTasks)
            {
                finalOptions[taskNum] =  tasks[taskNum][currentOption[taskNum]];
                taskNum++;
            }

            if(cGamepad.dpadLeftOnce() && currentOption[taskNum] > 0)
            {
                finalOptions[taskNum] = tasks[taskNum][currentOption[taskNum]];
                currentOption[taskNum] = currentOption[taskNum]-1;
            }
            if(cGamepad.dpadRightOnce() && currentOption[taskNum] < maxOptions)
            {
                finalOptions[taskNum] = tasks[taskNum][currentOption[taskNum]];
                currentOption[taskNum] = currentOption[taskNum]+1;
            }

            telemetry.addLine("(" + (taskNum+1) + ") " + taskName + ": " + tasks[taskNum][currentOption[taskNum]]);

            if(gamepad1.a)
            {
                flag = true;
            }

            telemetry.update();

        }



    }

    void initTasks()
    {
        // adding the options to color task
        colorTask[0] = "Blue";
        colorTask[1] = "Red";

        parkTask[0] = "Not Completely In ASU";
        parkTask[1] = "Completely In ASU";
        parkTask[2] = "Not Completely In WH";
        parkTask[3] = "Completely In WH";

        carouselTask[0] = "Yes";
        carouselTask[1] = "No";
    }
}