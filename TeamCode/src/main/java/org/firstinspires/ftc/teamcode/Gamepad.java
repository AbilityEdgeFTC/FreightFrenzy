package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


@TeleOp(name = "Gamepad OpMode")
public class Gamepad extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;

    double Power = .6;


    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        mFL = hardwareMap.get(DcMotor.class, "mFL");
        mFR = hardwareMap.get(DcMotor.class, "mFR");
        mBL = hardwareMap.get(DcMotor.class, "mBL");
        mBR = hardwareMap.get(DcMotor.class, "mBR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        mFL.setDirection(DcMotor.Direction.REVERSE);
        mFR.setDirection(DcMotor.Direction.FORWARD);
        mBL.setDirection(DcMotor.Direction.REVERSE);
        mBR.setDirection(DcMotor.Direction.FORWARD);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // Joysticks
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double leftPower_f;
            double leftPower_b;
            double rightPower_f;
            double rightPower_b;

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);
            //TODO: CHECK THIS AGAIN

            leftPower_f = (drive + strafe + twist) / denominator;
            leftPower_b = (drive - strafe + twist) / denominator;
            rightPower_f = (drive - strafe - twist) / denominator;
            rightPower_b = (drive + strafe - twist) / denominator;
            //TODO: CHECK THIS AGAIN

            mFL.setPower(leftPower_f);
            mBL.setPower(leftPower_b);
            mFR.setPower(rightPower_f);
            mBR.setPower(rightPower_b);

            
            // BUTTON Y
            if (gamepad1.y) {

            }

            // BUTTON A
            if (gamepad1.a) {

            }

            // BUTTON B
            if (gamepad1.b) {

            }

            // D-PAD DOWN
            if (gamepad1.dpad_down) {

            }

            // D-PAD UP
            if (gamepad1.dpad_up) {

            }

            // RIGHT BUMPER
            if (gamepad1.right_bumper) {

            }

            // LEFT BUMPER
            if (gamepad1.left_bumper) {
            }

        }
    }
}
    



