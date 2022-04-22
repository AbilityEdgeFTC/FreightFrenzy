package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class carusela{
    DcMotor mC;
    double angle = 0;
    double powerGiven = 0;
    public static double time = 3; //configerable
    double adding = 0;
    boolean flag = true;
    ElapsedTime elapsed1;
    ElapsedTime elapsed2;
    Gamepad gamepad1;
    cGamepad gamepad;
    public static double MAXPOWER = 1.0; //configerable
    public static double MINPOWER = 0.2;
    //boolean direction = false;
    Telemetry telemetry;
    // direction: false = right turn, true = left turn

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public carusela(HardwareMap hardwareMap, Gamepad gamepad1) {
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.gamepad1 = gamepad1;
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elapsed1 = new ElapsedTime();
        elapsed2 = new ElapsedTime();
        gamepad= new cGamepad(gamepad1);
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public carusela(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1) {
        this.mC = hardwareMap.get(DcMotor.class, "mC");
        this.gamepad1 = gamepad1;
        this.mC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
        elapsed1 = new ElapsedTime();
        elapsed2 = new ElapsedTime();
        gamepad= new cGamepad(gamepad1);
    }


    public int spinCarousel(){
        if(flag == true){
            elapsed1.reset();
            elapsed2.reset();
            flag = false;
        }
        adding = (MAXPOWER - MINPOWER) / time;
        powerGiven = MINPOWER;
        if(elapsed1.seconds() == 1.0){
            mC.setPower(powerGiven);
            if(powerGiven <= MAXPOWER)
                powerGiven += adding;
            displayTelemetry();
            if (elapsed2.seconds() == time){
                stopCarrosel();
                return 0;
            }
            elapsed1.reset();
        }
        return 1;
    }

    public void stopCarrosel(){
        mC.setPower(0);
        elapsed1.reset();
        elapsed2.reset();
    }
    // dispaly the data
    public void displayTelemetry(){
        if (telemetry != null){
            telemetry.addLine("Power at: " + powerGiven);
            telemetry.update();
        }
    }
}