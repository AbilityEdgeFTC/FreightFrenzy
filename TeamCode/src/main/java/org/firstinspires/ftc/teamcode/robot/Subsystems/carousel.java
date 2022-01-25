/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.Subsystems;

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
    public static double powerCarousel = 0.3;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public carousel(HardwareMap hardwareMap) {
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public carousel(HardwareMap hardwareMap, Telemetry telemetry) {
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    // spin carousel motor with power.
    public void spin(){
        mC.setPower(powerCarousel);
    }

    // spin carousel motor with power for 8 seconds long.
    public void spin(double seconds) throws InterruptedException {
        mC.setPower(powerCarousel);
        Thread.sleep((long)seconds*1000);
        stop();
    }

    // spin carousel motor with power, option for reversed is added.
    public void spin(boolean reverse){
        if(reverse){
            mC.setPower(-powerCarousel);
        }else{
            mC.setPower(powerCarousel);
        }
    }

    // spin carousel motor with power, option for reversed is added, for seconds long.
    public void spin(double seconds, boolean reverse) throws InterruptedException {
        if(reverse){
            mC.setPower(powerCarousel);
            Thread.sleep((long)seconds*1000);
            stop();
        }else{
            mC.setPower(-powerCarousel);
            Thread.sleep((long)seconds*1000);
            stop();
        }
    }

    // stop carousel motor.
    public void stop(){
        mC.setPower(0);
    }

    // display power of motor.
    public void displayTelemetry(){
        telemetry.addLine("Power at: " + powerCarousel);
        telemetry.update();
    }

}
