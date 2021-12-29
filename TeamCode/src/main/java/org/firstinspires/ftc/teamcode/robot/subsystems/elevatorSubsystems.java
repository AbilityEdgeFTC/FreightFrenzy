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

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class elevatorSubsystems{

    DcMotor mE;
    double power = 0;
    Telemetry telemetry = null;
    HardwareMap hw = null;
    public static int positionLevelOne;
    public static int positionLevelTwo;
    public static int positionLevelThree;

    // 2 constructors for 2 options, construct the carouselSubsystem with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO POWER, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public elevatorSubsystems(DcMotor mE, double power, int positionLevelOne, int positionLevelTwo, int positionLevelThree)
    {
        this.positionLevelOne = positionLevelOne;
        this.positionLevelTwo = positionLevelTwo;
        this.positionLevelThree = positionLevelThree;
        this.power = power;
        this.mE = mE;
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO POWER, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public elevatorSubsystems(DcMotor mE, double power, Telemetry telemetry, int positionLevelOne, int positionLevelTwo, int positionLevelThree) {
        this.positionLevelOne = positionLevelOne;
        this.positionLevelTwo = positionLevelTwo;
        this.positionLevelThree = positionLevelThree;
        this.power = power;
        this.mE = mE;
        this.telemetry = telemetry;

        this.mE = hw.get(DcMotor.class, "mE");
        this.mE.setDirection(DcMotorSimple.Direction.REVERSE);
        this.mE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    // set target position to zero pos(start pos), and while target position isn't required so keep going down.
    public void goToZeroPos() {
        mE.setTargetPosition(0);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(mE.getCurrentPosition() < positionLevelOne) {
            mE.setPower(power);
        }else {
            mE.setPower(0);
        }
    }

    // set target position to level one, and while target position isn't required so keep going up/down.
    public void goToLevelOne() {
        mE.setTargetPosition(positionLevelOne);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(mE.getCurrentPosition() < positionLevelOne) {
            mE.setPower(power);
        }else {
            mE.setPower(0);
        }

        // TODO: AFTER PLACING SERVO SO RETURN TO LEVEL 0
    }

    // set target position to level two, and while target position isn't required so keep going up/down.
    public void goToLevelTwo() {
        mE.setTargetPosition(positionLevelTwo);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(mE.getCurrentPosition() < positionLevelTwo) {
            mE.setPower(power);
        }else {
            mE.setPower(0);
        }

        // TODO: AFTER PLACING SERVO SO RETURN TO LEVEL 0
    }

    // set target position to level three, and while target position isn't required so keep going up.
    public void goToLevelThree() {
        mE.setTargetPosition(positionLevelThree);
        mE.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(mE.getCurrentPosition() < positionLevelThree) {
            mE.setPower(power);
        }else {
            mE.setPower(0);
        }

        // TODO: AFTER PLACING SERVO SO RETURN TO LEVEL 0

    }

    public int getPosition(){
        return mE.getCurrentPosition();
    }
    public void Up (){
        mE.setPower(power);
    }
    public void Down(){
        mE.setPower(-power);
    }



    // display power of motor, and its position.
    public void displayTelemetry(){
        telemetry.addLine("At: " + mE.getCurrentPosition());
        telemetry.addLine("Power at: " + power);
        telemetry.update();
    }

}
