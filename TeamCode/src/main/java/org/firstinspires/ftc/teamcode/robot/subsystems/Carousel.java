package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Carousel {

    DcMotor mC;
    NanoClock clock;
    double offset = 0;
    Telemetry telemetry;
    boolean newSpin = true;
    public static double MULTIPLYER = 5, MIN_POWER = 0.4, MAX_POWER = 1;
    Gamepad gamepad;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Carousel(HardwareMap hardwareMap, Gamepad gamepad1) {
        clock = NanoClock.system();
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mC.setDirection(DcMotorSimple.Direction.REVERSE);
        this.gamepad = gamepad1;
    }

    public Carousel(HardwareMap hardwareMap) {
        clock = NanoClock.system();
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mC.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public Carousel(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        clock = NanoClock.system();
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mC.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
        this.gamepad = gamepad1;
    }


    public void spinCarousel(boolean reverse){
        if(newSpin)
        {
            offset = clock.seconds();
            newSpin = false;
        }

        if(!reverse && !gamepad.dpad_down)
        {
            mC.setPower(MIN_POWER + ((getSeconds()) / MULTIPLYER));
        }
        else if(!gamepad.dpad_down)
        {
            mC.setPower(-(MIN_POWER + ((getSeconds()) / MULTIPLYER)));
        }

        if(gamepad.dpad_down && reverse)
        {
            mC.setPower(-MAX_POWER);
        }
        else if(gamepad.dpad_down)
        {
            mC.setPower(MAX_POWER);
        }

    }

    public void spinCarouselNoAccel(boolean reverse, double power){
        if(reverse)
        {
            mC.setPower(-power);
        }
        else
        {
            mC.setPower(power);
        }
    }

    public void displayTelemetry()
    {
        telemetry.addData("Power: ", mC.getPower());
        telemetry.addData("Seconds: ", getSeconds());
        telemetry.addData("Offset: ", offset);
        telemetry.update();
    }

    double getSeconds()
    {
        return clock.seconds() - offset;
    }

    public void stopCarousel(){
        mC.setPower(0);
        newSpin = true;
        offset = clock.seconds();
    }

    public void stopCarouselNoAccel(){
        mC.setPower(0);
    }

}