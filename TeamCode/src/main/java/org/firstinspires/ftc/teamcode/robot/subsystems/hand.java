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
    public static double /*level1Hub = 1, level2Hub = .8, */level3Hub = .63, levelSharedHub = .9;

    public enum HandPos
    {
        INTAKE,
        SHARED_HUB,
        ONE_HUB,
        TWO_HUB,
        THREE_HUB
    }

    public HandPos handPos = HandPos.INTAKE;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public hand(HardwareMap hardwareMap) {
        this.sR = hardwareMap.get(Servo.class, "sRight");
        this.sL = hardwareMap.get(Servo.class, "sLeft");
        this.sR.setDirection(Servo.Direction.REVERSE);
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public hand(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sL = hardwareMap.get(Servo.class, "sLeft");
        this.sR = hardwareMap.get(Servo.class, "sRight");
        this.sR.setDirection(Servo.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    public void goToPositions()
    {
        switch (handPos)
        {
            case INTAKE:
                intake();
                break;
            case SHARED_HUB:
                shared();
                break;
            case ONE_HUB:
                level1();
                break;
            case TWO_HUB:
                level2();
                break;
            case THREE_HUB:
                level3();
                break;
        }
    }

    public void intake()
    {
        handPos = HandPos.INTAKE;
        sL.setPosition(intakePos);
        sR.setPosition(intakePos);
    }

    public void shared()
    {
        handPos = HandPos.SHARED_HUB;
        sL.setPosition(levelSharedHub);
        sR.setPosition(levelSharedHub);
    }

    // spin dip servo to intake positions, and holding servo to hold position.
    public void level1()
    {
        handPos = HandPos.ONE_HUB;
//        sL.setPosition(level1Hub);
//        sR.setPosition(level1Hub);
        sL.setPosition(level3Hub);
        sR.setPosition(level3Hub);
    }

    public void level2()
    {
        handPos = HandPos.TWO_HUB;
//        sL.setPosition(level2Hub);
//        sR.setPosition(level2Hub);
        sL.setPosition(level3Hub);
        sR.setPosition(level3Hub);
    }

    public void level3()
    {
        handPos = HandPos.THREE_HUB;
        sL.setPosition(level3Hub);
        sR.setPosition(level3Hub);
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

    public HandPos getHandPos() {
        return handPos;
    }

    public double getPos() {
        return sL.getPosition();
    }


    public void setHandPos(HandPos handPos) {
        this.handPos = handPos;
    }
}
