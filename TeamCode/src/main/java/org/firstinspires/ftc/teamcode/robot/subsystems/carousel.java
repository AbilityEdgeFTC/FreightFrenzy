/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class carousel {

    //motor carousel
    DcMotor mC;
    Telemetry telemetry;
    int i = 0;
    public static double powerCarousel = 0.375, powerCarouselNoAccel = 0.3;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public carousel(HardwareMap hardwareMap) {
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
    }

    // spin carousel motor with power, option for reversed is added.
    public void spin(boolean reverse, boolean accel){
        if(reverse){
            if(accel)
            {
                mC.setPower(-(powerCarousel));
            }
            else
            {
                mC.setPower(-powerCarouselNoAccel);
            }

        }else{
            if(accel)
            {
                mC.setPower((powerCarousel));
            }
            else
            {
                mC.setPower(powerCarouselNoAccel);
            }
        }
    }

    // stop carousel motor.
    public void stop(){
        mC.setPower(0);
        i=0;
    }

    // display power of motor.
    public void displayTelemetry(){
        telemetry.addLine("Power at: " + powerCarousel);
        telemetry.update();
    }

}
