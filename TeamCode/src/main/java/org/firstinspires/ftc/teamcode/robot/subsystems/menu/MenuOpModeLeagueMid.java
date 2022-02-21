package org.firstinspires.ftc.teamcode.robot.subsystems.menu;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;

@TeleOp(name="Menu Autonmous League", group="Main")
public class MenuOpModeLeagueMid extends LinearOpMode
{
    //TODO: ADD TASKS WITH OPTIONS
    static taskObject autonomus =
            new taskObject(new String[]{"YES","NO"},"Run Autonomous");
    static taskObject side =
            new taskObject(new String[]{"LEFT","RIGHT"},"Alliance Side");
    static taskObject color =
            new taskObject(new String[]{"RED", "BLUE"},"Alliance Color");
    static taskObject vision =
            new taskObject(new String[]{"DUCK","TSE","NINE"},"Vision Detection");
    static taskObject delay =
            new taskObject(new String[]{"2","4","6","8","10","12","14","16","18","20","22","24","26"},"Start Delay");
    static taskObject numOfCycles =
            new taskObject(new String[]{"1","2","3","4","5"},"Num Of Cycles");
    static taskObject park =
            new taskObject(new String[]{"Warehouse Left","Warehouse Right","Warehouse Half","Alliance Shipping Unit"},"Park In");

    //all tasks taskObject array:
    public static taskObject[] tasks = {autonomus,side,color,vision,delay,numOfCycles,park};
    //final list string array
    public static String[] finalOptions = {};

    int maxTasks = 0, maxOptions = 0, taskNum = 0,optionNum = 0;
    boolean flag = false;

    @Override
    public void runOpMode()
    {
        cGamepad cGamepad = new cGamepad(gamepad1);


        // set every current option by the number of tasks to 0
        /*
        for(int i = 0; i < tasks.length; i++)
        {
            currentOption[i] = 0;
        }
        */



        // max tasks is the size of the list of tasks - 1.
        maxTasks = tasks.length;

        // wait for the game to start (driver presses PLAY)
        waitForStart();

        // run while player didn't press stop
        while (opModeIsActive() && !flag)
        {

            cGamepad.update();

            // setting the max options to the number of options for current task - 1
            maxOptions = tasks[taskNum].size();


            /**up and down(task pick)**/
            if(cGamepad.dpadUpOnce() && taskNum > 0)
                finalOptions[taskNum] = tasks[taskNum].getOption(optionNum);
                taskNum--;
                maxOptions = tasks[taskNum].size();
            }
            if(cGamepad.dpadDownOnce() && taskNum < maxTasks)
            {
                finalOptions[taskNum] = tasks[taskNum].getOption(optionNum);
                taskNum++;
                maxOptions = tasks[taskNum].size();
            }




            /**left and right (option pick)**/
            if(cGamepad.dpadLeftOnce() && optionNum > 0)
            {
                optionNum--;

            }
            if(cGamepad.dpadRightOnce() && optionNum < maxOptions)
            {
                optionNum++;
            }

            /**Telementry **/
            telemetry.addLine("(" + (taskNum+1) + ") " + tasks[taskNum].getName() + ": " + tasks[taskNum].getOption(optionNum));

            //-----------------------------------------------------------------------------------------------------------------------------------------------
            if(gamepad1.a)
            {
                for(int i = 0; i < finalOptions.length; i++)
                {
                    switch(i)
                    {
                        case 0:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("autoTask.txt"), finalOptions[i]);
                            break;
                        case 1:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("localizerTask.txt"), finalOptions[i]);
                            break;
                        case 2:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("allianceTask.txt"), finalOptions[i]);
                            break;
                        case 3:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("startPosTask.txt"), finalOptions[i]);
                            break;
                        case 4:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("startDelayTask.txt"), finalOptions[i]);
                            break;
                        case 5:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("carouselTask.txt"), finalOptions[i]);
                            break;
                        case 6:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("placeFreightAtTask.txt"), finalOptions[i]);
                            break;
                        case 7:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("collectFreightTask.txt"), finalOptions[i]);
                            break;
                        case 8:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("numOfFreightTask.txt"), finalOptions[i]);
                            break;
                        case 9:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("parkInTask.txt"), finalOptions[i]);
                            break;
                        case 10:
                            ReadWriteFile.writeFile(AppUtil.getInstance().getSettingsFile("parkTypeTask.txt"), finalOptions[i]);
                            break;
                    }
                }
                flag = true;
            }

            telemetry.update();

        }




}