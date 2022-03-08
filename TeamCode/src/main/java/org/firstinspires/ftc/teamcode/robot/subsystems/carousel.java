/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class carousel {

    //motor carousel
    CRServo sCL, sCR;
    Telemetry telemetry;
    public static double powerCarousel = 1;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public carousel(HardwareMap hardwareMap) {
        this.sCL = hardwareMap.get(CRServo.class, "sCL");
        this.sCR = hardwareMap.get(CRServo.class, "sCR");
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sCL = hardwareMap.get(CRServo.class, "sCL");
        this.sCR = hardwareMap.get(CRServo.class, "sCR");
        this.telemetry = telemetry;
    }

    // spin carousel motor with power, option for reversed is added.
    public void spin(boolean reverse){
        if(reverse){
            sCL.setPower(-powerCarousel);
        }else{
            sCR.setPower(powerCarousel);
        }
    }

    // stop carousel motor.
    public void stop(){
        sCL.setPower(0);
        sCR.setPower(0);
    }

    // display power of motor.
    public void displayTelemetry(){
        telemetry.addLine("Power at: " + powerCarousel);
        telemetry.update();
    }

}
