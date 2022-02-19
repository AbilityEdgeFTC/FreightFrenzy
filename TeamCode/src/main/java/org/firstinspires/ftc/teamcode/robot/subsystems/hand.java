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
    public static double intakePos = 0, level1 = 0, level2 = 0, level3 = 0;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public hand(HardwareMap hardwareMap) {
        this.sL = hardwareMap.get(Servo.class, "sLH");
        this.sR = hardwareMap.get(Servo.class, "sRH");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public hand(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sL = hardwareMap.get(Servo.class, "sLH");
        this.sR = hardwareMap.get(Servo.class, "sRH");
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
        sL.setPosition(level1);
        sR.setPosition(level1);
    }

    public void level2()
    {
        sL.setPosition(level2);
        sR.setPosition(level2);
    }

    public void level3()
    {
        sL.setPosition(level3);
        sR.setPosition(level3);
    }

    public void moveTo(double position)
    {
        sL.setPosition(position);
        sR.setPosition(position);
    }

    // display position of servo's.
    public void displayTelemetry(){
        telemetry.addData("Servo left hand at", sL.getPosition());
        telemetry.addData("Servo right hand at", sR.getPosition());
        telemetry.update();
    }

}
