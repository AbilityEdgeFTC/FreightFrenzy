/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Millie gamepadE OpMode")
public class GamepadEncoders extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;


    static final double MOTOR_TICK_COUNTS = 537.7;

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


        mFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // How many turns do I need the wheels to go 4 inches?
        // the distance you drive with one turn of wheel is the circumfernce of the wheel.
        double circumfernce = 3.14 * 2.473; // pi * diameter
        double rotationNeeded = 4 / circumfernce;
        int encoderDrivingTarget = (int) (rotationNeeded * 537.7); // I don't need to look at the existing
        // encoder counts because I just reset the encoder counts above 0.

        // set the target positions
        mFL.setTargetPosition(encoderDrivingTarget);
        mFR.setTargetPosition(encoderDrivingTarget);
        mBR.setTargetPosition(encoderDrivingTarget);
        mBL.setTargetPosition(encoderDrivingTarget);

        // set the power desired for the motors
        mFL.setPower(.6);
        mFR.setPower(.6);
        mBR.setPower(.6);
        mBL.setPower(.6);

        // set the motors to RUN_TO_POSITION
        mFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // essentially do nothing while you wait for the robot to finish driving to the position
        while (mBR.isBusy()) mBL.isBusy();
        {
            telemetry.addData("Status", "Driving 4 inches");
            telemetry.update();
            while (mFR.isBusy()) mFL.isBusy();
            {
                telemetry.addData("Status", "Driving 4 inches");
                telemetry.update();
            }
        }

        // stop all motors
        mFL.setPower(0);
        mFR.setPower(0);
        mBR.setPower(0);
        mBL.setPower(0);

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}













