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
    double powerGiven = 0;
    public static double addingBy = 0.25;
    public static double MAX_POWER = 0.5; //configerable
    public static double MIN_POWER = 0.3;
    public static double delay = 1;
    public static double startAccel = .19;
    NanoClock clock;
    double offset = 0;
    Telemetry telemetry;
    boolean newSpin = true;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Carousel(HardwareMap hardwareMap, Gamepad gamepad1) {
        clock = NanoClock.system();
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mC.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Carousel(HardwareMap hardwareMap) {
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
    }


    public void spinCarousel(boolean reverse){

        if(getSeconds() > delay && newSpin)
        {
            powerGiven += startAccel;
        }
        else if(powerGiven < MAX_POWER)
        {
            powerGiven += addingBy;
        }

        if(reverse)
        {
            mC.setPower(-powerGiven);
        }
        else
        {
            mC.setPower(powerGiven);
        }
    }

    public void spinCarouselNoAccel(boolean reverse){
        if(reverse)
        {
            mC.setPower(-MIN_POWER);
        }
        else
        {
            mC.setPower(MIN_POWER);
        }
    }

    public void displayTelemetry()
    {
        telemetry.addData("Power: ", powerGiven);
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
        powerGiven = MIN_POWER;
    }

    public void stopCarouselNoAccel(){
        mC.setPower(0);
    }

}