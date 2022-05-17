package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Carousel {

    DcMotor mC;
    double powerGiven = 0;
    double addingBy = 0.01;
    public static double MAX_POWER = 0.6; //configerable
    public static double MIN_POWER = 0.2;
    public static double delay = 1;
    NanoClock clock;
    double offset = 0;
    //boolean direction = false;
    Telemetry telemetry;
    // direction: false = right turn, true = left turn

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public Carousel(HardwareMap hardwareMap, Gamepad gamepad1) {
        clock = NanoClock.system();
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public Carousel(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        clock = NanoClock.system();
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
    }


    public void spinCarousel(){

        if(getSeconds() > delay && powerGiven < MAX_POWER)
        {
            powerGiven += addingBy;
            offset = clock.seconds();
        }

        mC.setPower(powerGiven);
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
        offset = clock.seconds();
        powerGiven = MIN_POWER;
    }

}