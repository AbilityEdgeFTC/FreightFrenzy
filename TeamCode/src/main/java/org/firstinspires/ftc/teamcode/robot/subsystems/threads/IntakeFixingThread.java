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
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;

@Config
public class IntakeFixingThread extends Thread {

    intake intake;
    FreightSensor freightSensor;
    Telemetry telemetry;
    public static boolean exit = false;
    public static boolean spinIntake = false;

    public IntakeFixingThread(HardwareMap hw, Telemetry telemetry) {
        intake = new intake(hw);
        freightSensor = new FreightSensor(hw);
        this.telemetry = telemetry;
    }

    // called when tread.start is called. thread stays in loop to do what it does until exit is
    // signaled by main code calling thread.interrupt.
    @Override
    public void run() {
        try {
            while (!exit)
            {
                if(freightSensor.freightOn())
                {
                    intake.intakeBackward();
                    Thread.sleep(1500);
                }

                while(freightSensor.freightOn())
                {
                    intake.stop();
                }

                while (!freightSensor.freightOn() && spinIntake)
                {
                    intake.intakeForward();
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
}