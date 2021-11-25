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

package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class carouselSubsystem{

    //motor carousel
    //DcMotor mC;
    CRServo sC;
    double power;
    HardwareMap hw;
    Telemetry telemetry;

    // 2 constructors for 2 options, construct the carouselSubsystem with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public carouselSubsystem(CRServo sC, double power, HardwareMap hw) {
        this.sC = sC;
        this.power = power;
        this.hw = hw;

        this.sC = hw.get(CRServo.class, "sC");
        sC = hw.get(CRServo.class, "sC");
        //sC.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public carouselSubsystem(CRServo sC, double power, HardwareMap hw, Telemetry telemetry) {
        this.sC = sC;
        this.power = power;
        this.hw = hw;
        this.telemetry = telemetry;

        this.sC = hw.get(CRServo.class, "sC");
        sC = hw.get(CRServo.class, "sC");
        //sC.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // spin carousel motor with power.
    public void spinCarouselMotor(){
        sC.setPower(power);
    }

    // spin carousel motor with power for seconds long.
    public void spinCarouselMotor(double seconds) throws InterruptedException {
        sC.setPower(power);
        Thread.sleep((long)seconds*1000);
        stopCarouselMotor();
    }

    // spin carousel motor with -power.
    public void spinCarouselMotor(boolean reverse){
        if(reverse){
            sC.setPower(-power);
        }else{
            sC.setPower(power);
        }
    }

    // spin carousel motor with -power for seconds long.
    public void spinCarouselMotor(double seconds, boolean reverse) throws InterruptedException {
        if(reverse){
            sC.setPower(power);
            Thread.sleep((long)seconds*1000);
            stopCarouselMotor();
        }else{
            sC.setPower(-power);
            Thread.sleep((long)seconds*1000);
            stopCarouselMotor();
        }
    }

    // stop carousel motor.
    public void stopCarouselMotor(){
        sC.setPower(0);
    }

    // display power of motor.
    public void displayTelemetry(){
        telemetry.addLine("Power at: " + power);
        telemetry.update();
    }

}
