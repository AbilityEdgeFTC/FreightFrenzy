package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Subsystems.cGamepad;

import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.allianceTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.autoTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.carouselTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.collectFreightTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.currentOption;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.finalOptions;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.numOfFreightTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.parkInTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.parkTypeTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.placeFreightAtTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.startDelayTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.startPosTask;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.taskName;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.tasks;
import static org.firstinspires.ftc.teamcode.robot.Subsystems.valueStorage.tasksName;

@TeleOp(name="Menu Autonmous", group="Main")
public class MenuOpMode extends LinearOpMode
{

    int maxTasks = 0, maxOptions = 0, taskNum = 0;
    boolean flag = false;

    @Override
    public void runOpMode()
    {
        cGamepad cGamepad = new cGamepad(gamepad1);
        initTasks();

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
        tasks[0] = autoTask;
        tasks[1] = allianceTask;
        tasks[2] = startPosTask;
        tasks[3] = startDelayTask;
        tasks[4] = carouselTask;
        tasks[5] = collectFreightTask;
        tasks[6] = numOfFreightTask;
        tasks[7] = placeFreightAtTask;
        tasks[8] = parkInTask;
        tasks[9] = parkTypeTask;

        tasksName[0] = "Run Auto";
        tasksName[1] = "Alliance";
        tasksName[2] = "Start Position";
        tasksName[3] = "Start Delay";
        tasksName[4] = "Spin Carousel";
        tasksName[5] = "Collect And Place Additional Freight";
        tasksName[6] = "Number Of Freight To Collect";
        tasksName[7] = "Place Freight In";
        tasksName[8] = "Park In";
        tasksName[9] = "Park Completely";

        initOptions();
    }

    void initOptions()
    {
        autoTask[0] = "YES";
        autoTask[1] = "NO";

        allianceTask[0] = "Red";
        allianceTask[1] = "Blue";

        startPosTask[0] = "Right";
        startPosTask[1] = "Left";

        for(int i = 0; i < 30; i++)
        {
            startDelayTask[i] = "" + i+1;
        }

        carouselTask[0] = "YES";
        carouselTask[1] = "NO";

        collectFreightTask[0] = "YES";
        collectFreightTask[1] = "NO";

        for(int i = 0; i < 10; i++)
        {
            numOfFreightTask[i] = "" + i+1;
        }

        placeFreightAtTask[0] = "LEFT";
        placeFreightAtTask[1] = "RIGHT";
        placeFreightAtTask[2] = "FRONT";
        placeFreightAtTask[3] = "BACK";

        parkInTask[0] = "Alliance Shipping Unit";
        parkInTask[1] = "Warehouse";

        parkTypeTask[0] = "Completely";
        parkTypeTask[0] = "Not Completely";

    }
}