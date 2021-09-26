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
import com.qualcomm.robotcore.hardware.HardwareMap;
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

@TeleOp(name="Millie gamepadJ OpMode")
public class GamepadTaskJ extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor mFL = null;
    private DcMotor mFR = null;
    private DcMotor mBL = null;
    private DcMotor mBR = null;
    private Servo arm = null;
    private CRServo sR = null;

    double ARM_Min_Range = 0.0;
     double ARM_Max_Range = 1.0;


    double Power = .6;
    double sRPower = .7;


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
        arm = hardwareMap.get(Servo.class, "arm");
        sR = hardwareMap.get(CRServo.class, "sR");
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


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double leftPower_f;
            double leftPower_b;
            double rightPower_f;
            double rightPower_b;

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_y;
            double twist = gamepad1.right_stick_x;

            leftPower_f = Range.clip(drive + strafe + twist, -Power, Power);
            leftPower_b = Range.clip(drive - strafe + twist, -Power, Power);
            rightPower_f = Range.clip(drive - strafe - twist, -Power, Power);
            rightPower_b = Range.clip(drive + strafe - twist, -Power, Power);

            mFL.setPower(leftPower_f);
            mFR.setPower(rightPower_f);
            mBL.setPower(leftPower_b);
            mBR.setPower(rightPower_b);

            if (gamepad1.dpad_right) {
                arm.setPosition(ARM_Max_Range);
            } else if (gamepad1.dpad_left) {
                arm.setPosition(ARM_Min_Range);
            }
            if (gamepad1.left_bumper) {
                sR.setPower(-sRPower);
            } else if (gamepad1.right_bumper) {
                sR.setPower(sRPower);
            }
                else {
                    sR.setPower(0);
                }
            }
        }
    }









