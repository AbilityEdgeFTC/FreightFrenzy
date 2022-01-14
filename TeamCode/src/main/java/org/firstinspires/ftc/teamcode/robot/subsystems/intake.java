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

package org.firstinspires.ftc.teamcode.robot.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class intake {

    DcMotor mI;
    double power;
    Telemetry telemetry;

    /**
     * constructor for intake
     * @param mI the intake motor
     * @param power the power to give the motor
     */
    public intake(DcMotor mI, double power) {
        this.power = power;
        this.mI = mI;
    }

    /**
     * constructor for intake
     * @param mI the intake motor
     * @param power the power to give the motor
     * @param telemetry the telemetry object from the opmode
     */
    public intake(DcMotor mI, double power, Telemetry telemetry) {
        this.power = power;
        this.mI = mI;
        this.telemetry = telemetry;
    }

    /**
     * set power to mI
     * setting the power given from the constructor to the mI
     */
    public void intakeForward(){
        mI.setPower(power);
    }

    /**
     * set -power to mI
     * setting the power in a negative value given from the constructor to the mI
     */
    public void intakeBackward(){
        mI.setPower(-power);
    }

    /**
     * set power 0 to mI
     * stopping the mI motor
     */
    public void stop(){
        mI.setPower(0);
    }

    /**
     * displaying intake motor power
     */
    public void displayTelemetry(){
        telemetry.addData("Intake motor power at", power);
        telemetry.update();
    }

}