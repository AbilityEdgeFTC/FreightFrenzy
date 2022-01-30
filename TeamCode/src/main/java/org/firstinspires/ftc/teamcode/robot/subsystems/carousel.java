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
    double i = 0;
    public static double powerCarousel = 0.1, addBy = 0.0001;

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
        mC.setPower(powerCarousel + i * addBy);
        i++;
    }

    // spin carousel motor with power for 8 seconds long.
    public void spin(double seconds) throws InterruptedException {
        mC.setPower(powerCarousel + i * addBy);
        i++;
        Thread.sleep((long)seconds*1000);
        stop();
    }

    // spin carousel motor with power, option for reversed is added.
    public void spin(boolean reverse){
        if(reverse){
            mC.setPower(-(powerCarousel + i * addBy));
            i++;
        }else{
            mC.setPower(powerCarousel + i * addBy);
            i++;
        }
    }

    // spin carousel motor with power, option for reversed is added, for seconds long.
    public void spin(double seconds, boolean reverse) throws InterruptedException {
        if(reverse){
            mC.setPower(-(powerCarousel + i * addBy));
            i++;
            Thread.sleep((long)seconds*1000);
            stop();
        }else{
            mC.setPower(powerCarousel + i * addBy);
            i++;
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
