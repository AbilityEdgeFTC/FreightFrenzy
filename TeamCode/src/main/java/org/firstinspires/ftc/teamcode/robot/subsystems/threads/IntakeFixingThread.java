/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */
package org.firstinspires.ftc.teamcode.robot.subsystems.threads;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightSensor;
import org.firstinspires.ftc.teamcode.robot.subsystems.SensorColor;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
public class IntakeFixingThread extends Thread {

    intake intake;
    SensorColor colorSensor;
    public static boolean exit = false;
    public static boolean spinIntake = false;
    public static double MIN_CM = 0, MAX_CM = 0;
    public static double MIN_H = 0, MAX_H = 0;
    public static double MIN_S = 0, MAX_S = 0;
    public static double MIN_V = 0, MAX_V = 0;
    public boolean override = false;


    public IntakeFixingThread(HardwareMap hw) {
        intake = new intake(hw);
        colorSensor = new SensorColor(hw);
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            while (!exit)
            {
                if(!override)
                {
                    if(withFreight())
                    {
                        intake.intakeBackward();
                        Thread.sleep(1500);
                    }

                    while(withFreight())
                    {
                        intake.stop();
                    }

                    while (!withFreight() && spinIntake)
                    {
                        intake.intakeForward();
                    }
                }
            }
            Thread.currentThread().interrupt();
        }
        // an error occurred in the run loop.
        catch (Exception e) {
        }
    }

    public static void exitThread()
    {
        exit = true;
    }

    public boolean withFreight()
    {

        if(distanceIsMathcing() || colorIsMatching())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public boolean colorIsMatching()
    {
        float[] color = colorSensor.getHSV();

        if((color[0] >= MIN_H && color[0] <= MAX_H) && (color[1] >= MIN_S && color[1] <= MAX_S) && (color[2] >= MIN_V && color[2] <= MAX_V))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public boolean distanceIsMathcing()
    {
        double distance = colorSensor.getCM();
        if(distance >= MIN_CM && distance <= MAX_CM)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    public void setOverride(boolean override) {
        this.override = override;
    }
}