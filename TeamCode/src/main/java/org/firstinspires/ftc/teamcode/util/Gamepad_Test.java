/**
 *
 *
 *
 *
 * This file contains an gamepad OpMode for the 2021 Ultimate Goal Season, made by Ability Edge.
 *
 *
 *
 *
 */


package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.teamcode.subsystems.elevatorSubsystems;


@TeleOp(name="Gamepad Test", group="Gamepad")
public class Gamepad_Test extends LinearOpMode {

    DcMotor mE = null;

    /**
     * these 7 variables are for all of our 4 motors(leftMotor_f, leftMotor_b, rightMotor_f, rightMotor_b) for our drivetrain and
     the power to give to the drivetrain's motors. there also a minPower and maxPower
     in case you need to slow down the robot or move faster.
     */
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;


    private double power = .8;
    private double minPower=.5;
    private static double epower=0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        // START OF INIT



        /**
         * these 4 variables are for our drivetrain motors, one for each ide(right left, back front).
         */
        leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b  = hardwareMap.get(DcMotor.class, "mBL");
        rightMotor_f  = hardwareMap.get(DcMotor.class, "mFR");
        rightMotor_b  = hardwareMap.get(DcMotor.class, "mBR");

        /**
         *  Reverse the motors who are backwards. in our robot, the left motors are reversed
         but the right motors are in the default forwards direction.
         */
        leftMotor_f.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor_b.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor_f.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor_b.setDirection(DcMotorSimple.Direction.FORWARD);


        /**
         * clearing all telemetry, printing Ready! and updating the text.
         */
        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        //elevatorSubsystems elevator = new elevatorSubsystems(mE, epower, hardwareMap, telemetry);
        // END OF INIT
        waitForStart();
        // START OF THE CODE, RUNS ONE TIME
        if (isStopRequested()) return;


        while(opModeIsActive()){
            // RUNS WHILE NO ONE PRESSED STOP.

            // Setup a variable for each drive wheel.
            double leftPower_f;
            double leftPower_b;
            double rightPower_f;
            double rightPower_b;

            // Here we declare the gamepad values given from the gamepad.
            // After we use range.clip(which sets a minimum and maximum for the gamepad values.
            // And lastly by the mechanum wheels gamepad formula we give every wheel a value.
            double drive = -gamepad1.left_stick_y; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x; // Counteract imperfect strafing
            double twist = gamepad1.right_stick_x;

            leftPower_f = Range.clip(drive + strafe + twist, -power, power);
            leftPower_b   = Range.clip(drive - strafe + twist, -power, power);
            rightPower_f = Range.clip(drive - strafe - twist, -power, power);
            rightPower_b   = Range.clip(drive + strafe - twist, -power, power);

            // Send calculated power to wheels
            leftMotor_f.setPower(leftPower_f);
            leftMotor_b.setPower(leftPower_b);
            rightMotor_f.setPower(rightPower_f);
            rightMotor_b.setPower(rightPower_b);

            /**
             * if the joysticks are pressed, so the power is increased or decreased, depends on
             the left or right joystick.
             */
            if(gamepad1.left_stick_button){
                power = minPower;
            }else if(gamepad1.right_stick_button){
                power = minPower;
            }else{
                power = .8;
            }
            if(gamepad1.a){
//                elevator.goToLevelOne();
            }else if(gamepad1.b){
  //              elevator.goToLevelTwo();
            }else if(gamepad1.x){
    //            elevator.goToLevelThree();
            }else if(gamepad1.y){
      //          elevator.goToZeroPos();
            }

        //    elevator.displayTelemetry();

            // Update the telemetry on the phone.
            telemetry.update();

        }
    }
}
