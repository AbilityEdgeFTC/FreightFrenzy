/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class hand {

    Servo sL, sR;
    Telemetry telemetry;
    public static double intakePos = 0.07;
    public static double level1Hub = 0.95, level2Hub = 0.85, level3Hub = .63, levelSharedHub = 0.7;



    public static Elevator.ElevatorLevel elevatorLevel = Elevator.elevatorLevel;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public hand(HardwareMap hardwareMap) {
        this.sL = hardwareMap.get(Servo.class, "sLeft");
        this.sR = hardwareMap.get(Servo.class, "sRight");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public hand(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sL = hardwareMap.get(Servo.class, "sLeft");
        this.sR = hardwareMap.get(Servo.class, "sRight");
        this.telemetry = telemetry;
    }

    public void intake()
    {
        sL.setPosition(intakePos);
        sR.setPosition(intakePos);
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public void level1()
    {
        switch (elevatorLevel)
        {
            case SHARED_HUB:
                sL.setPosition(levelSharedHub);
                sR.setPosition(levelSharedHub);
                break;
            case HUB_LEVEL1:
                sL.setPosition(level1Hub);
                sR.setPosition(level1Hub);
                break;
        }
    }

    public void level2()
    {
        sL.setPosition(level2Hub);
        sR.setPosition(level2Hub);
    }

    public void level3()
    {
        sL.setPosition(level3Hub);
        sR.setPosition(level3Hub);
    }



    public void moveTo(double position)
    {
        sL.setPosition(position);
        sR.setPosition(position);
    }

    public void updateElevatorLevel(Elevator.ElevatorLevel elevatorLevel)
    {
        this.elevatorLevel = elevatorLevel;
    }

    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addData("Servo left hand at", sL.getPosition());
        telemetry.addData("Servo right hand at", sR.getPosition());
        telemetry.update();
    }

}
