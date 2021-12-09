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


package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@Config
@TeleOp(name="Gamepad OnePad", group="States")
public class TeleOpGPoneGP extends LinearOpMode {


    /**
     * these 7 variables are for all of our 4 motors(leftMotor_f, leftMotor_b, rightMotor_f, rightMotor_b) for our drivetrain and
     the power to give to the drivetrain's motors. there also a minPower and maxPower
     in case you need to slow down the teleop or move faster.
     */
    private DcMotor leftMotor_f = null;
    private DcMotor leftMotor_b = null;
    private DcMotor rightMotor_f = null;
    private DcMotor rightMotor_b = null;
    private static double power = .5;
    private double minPower = .5;
    private double maxPower = 1;


    /**
     * these 3 variables are for our intake motor,
     the power(powerIntake) to give to the intake motor(mIntake), and if the intake should be on or not.
     */
    private DcMotorEx mIntake;
    private double powerIntake = 1;
    private boolean powerOnIntake = false;


    /**
     * these 4 variables are for our shooting mechanism.
     * the MIN_POS is 0.3 for the default position of the servo, which is not pushing the rings to the
     shooter. the pushing position is MAX_POS.
     the servoPS is the servo that pushes the rings from the cartridge to the shooting motors and shooting the ring out of the teleop.
     * the timeToShoot variable is the amount of milliseconds between each ring shot while shooting 3 rings. in this case
     we will have 0.2 seconds delay till the next ring is shot out of the teleop.
     */
    Servo servoPS;
    double MIN_POS     =  0.3;     // Minimum rotational position
    double MAX_POS =  0.0;     // Maximum rotational position
    long timeToShoot = 200;


    /**
     * these 2 variables are for our wabble arm mechanism.
     * it has a motor(mElevator) which acts like an arm that goes up and down
     and the amount of power to give to the motor, which is powerWabble.
     */
    DcMotor mElevator;
    private double powerWabble = 1;


    /**
     * these 4 variables are for our wabble hand mechanism.
     * it has a servo(servoWobble), that has a minimum(MIN_POSwabble) and maximum(MAX_POSwabble) value that is can rotate to.
     * The minimum rotational position is when the hand is open and the maximum rotational position
     is when the hand is closed, the default position(position) is the hand opened. //TODO: CHECK IF THE HAND IS CLOSED OR OPENED BY THE DEFAULT.
     */
    Servo servoWabble;
    double MIN_POSwabble = 0.0;     // Minimum rotational position
    double MAX_POSwabble = 0.9;     // Maximum rotational position
    double  position = MIN_POSwabble;


    /**
     * these 4 variables are for our shooting mechanism.
     2 motors, left and right Shoot(leftShoot & rightShoot( both running on the powerShoot variable. these 2 motors spin very
     fast and shoot the rings out of the teleop. the powerOnShoot boolean is to see if the shooting motors should turn or not.
     */
    DcMotor leftShoot;
    DcMotor rightShoot;
    double powerShoot = .6;
    boolean powerOnShoot = false;


    @Override
    public void runOpMode() throws InterruptedException {
        // START OF INIT


        /**
         * these 2 variables are for our wabble mechanism, its hand and arm.
         */
        servoWabble = hardwareMap.get(Servo.class, "sW");
        mElevator = hardwareMap.get(DcMotor.class, "mW");


        /**
         * these 4 variables are for our drivetrain motors, one for each ide(right left, back front).
         */
        leftMotor_f  = hardwareMap.get(DcMotor.class, "mFL");
        leftMotor_b  = hardwareMap.get(DcMotor.class, "mBL");
        rightMotor_f  = hardwareMap.get(DcMotor.class, "mFR");
        rightMotor_b  = hardwareMap.get(DcMotor.class, "mBR");

        /**
         *  Reverse the motors who are backwards. in our teleop, the left motors are reversed
         but the right motors are in the default forwards direction.
         */
        leftMotor_f.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor_b.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor_f.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor_b.setDirection(DcMotorSimple.Direction.FORWARD);

        /**
         * these 3 variables are for our shooting mechanism, the servo pushing the rings from the
         magazine to the shooting motors, the 2 shooting motors.
         * we need to reverse the right shoot motor of the shooting mechanism.
         */
        servoPS = hardwareMap.get(Servo.class, "sPS");
        leftShoot = hardwareMap.get(DcMotorEx.class, "mLS");
        rightShoot = hardwareMap.get(DcMotorEx.class, "mRS");
        rightShoot.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
         * setting our intake motor to its configuration in the driver station app.
         */
        mIntake = hardwareMap.get(DcMotorEx.class, "mI");


        /**
         * clearing all telemetry, printing Ready! and updating the text.
         */
        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

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
                power = maxPower;
            }else{
                power = .8;
            }

            /**
             * all lines of code down here have the same idea, if a button on the gamepad 1
             is pressed so it will or power a motor, and/or move a servo to a position.
             */


            if(gamepad1.dpad_up){
                mElevator.setPower(powerWabble);
            }else if(gamepad1.dpad_down){
                mElevator.setPower(-powerWabble);
            }else{
                mElevator.setPower(0);
            }

            if(gamepad1.b){
                position = MAX_POSwabble ;
            }else if(gamepad1.x){
                position = MIN_POSwabble ;
            }

            if(gamepad1.y){
                powerOnShoot =! powerOnShoot;
            }else if(gamepad1.a){
                powerOnIntake =! powerOnIntake;
            }

            if(powerOnIntake){
                mIntake.setPower(powerIntake);
            }else{
                mIntake.setPower(0);
            }

            if(powerOnShoot){
                leftShoot.setPower(powerShoot);
                rightShoot.setPower(powerShoot);
                power = 0.5;
            }else{
                leftShoot.setPower(0);
                rightShoot.setPower(0);
                power = 0.9;
            }


            /**
             * if the right bumper on the gamepad1 is pressed, it will shoot 3 rings with a
             250-300 millisecond delay between each ring that was shot.
             * if the dpad-down on the gamepad1 is pressed, it will shoot one ring, with a
             200 millisecond delay.
             */

            if(gamepad1.right_bumper){
                servoPS.setPosition(MIN_POS);
                Thread.sleep(timeToShoot + 100);
                servoPS.setPosition(MAX_POS);
                Thread.sleep(timeToShoot);
                servoPS.setPosition(MIN_POS + 50);
                Thread.sleep(timeToShoot);
                servoPS.setPosition(MAX_POS);
                Thread.sleep(timeToShoot);
                servoPS.setPosition(MIN_POS);
                Thread.sleep(timeToShoot + 100);
                servoPS.setPosition(MAX_POS);
                Thread.sleep(timeToShoot);
                servoPS.setPosition(MIN_POS);
                Thread.sleep(timeToShoot + 50);
            }

            if(gamepad1.left_bumper){
                servoPS.setPosition(MIN_POS);
                Thread.sleep(timeToShoot);
                servoPS.setPosition(MAX_POS);
                Thread.sleep(timeToShoot);
                servoPS.setPosition(MIN_POS);
                Thread.sleep(timeToShoot);
            }



            // Set the servo to the new position and wait for it to turn to the position.
            servoWabble.setPosition(position);
            Thread.sleep(timeToShoot / 2);
            // Update the telemetry on the phone.
            telemetry.update();

        }
    }
}
