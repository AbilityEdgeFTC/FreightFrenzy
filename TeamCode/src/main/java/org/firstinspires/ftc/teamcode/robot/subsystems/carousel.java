/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class carousel {

    //motor carousel
    CRServo sR;
    CRServo sL;
    Telemetry telemetry;
    public static double powerCarousel = 1;
    public static boolean reverse = false;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public carousel(HardwareMap hardwareMap) {
        this.sR = hardwareMap.get(CRServo.class, "sRC");
        this.sL = hardwareMap.get(CRServo.class, "sLC");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sR = hardwareMap.get(CRServo.class, "sRC");
        this.sL = hardwareMap.get(CRServo.class, "sLC");
        this.telemetry = telemetry;
    }

    // spin carousel motor with power, option for reversed is added.
    public void spin(boolean requestReversed){
        if(reverse)
        {
            if(requestReversed){
                sL.setPower(-powerCarousel);
                sR.setPower(powerCarousel);
            }else{
                sL.setPower(powerCarousel);
                sR.setPower(-powerCarousel);
            }
        }
        else
        {
            if(requestReversed){
                sL.setPower(powerCarousel);
                sR.setPower(-powerCarousel);
            }else{
                sL.setPower(-powerCarousel);
                sR.setPower(powerCarousel);
            }
        }
    }

    // stop carousel motor.
    public void stop(){
        sL.setPower(0);
        sR.setPower(0);
    }

    // display power of motor.
    public void displayTelemetry(){
        telemetry.addLine("Power at: " + powerCarousel);
        telemetry.update();
    }

}
