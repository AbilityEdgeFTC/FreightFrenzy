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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class dip {
    // TODO: hand holding 1 pushing .4
    //servo intake
    Servo sD, sH;
    public static double intakePosition = .42, releasingPosition = .25, holdingPosition = 1, pushingPosition = .4;
    Telemetry telemetry;

    // 2 constructors for 2 options, construct the carousel with and without telementry.
    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, AND HARDWAREMAP.  */
    public dip(HardwareMap hardwareMap) {
        this.sD = hardwareMap.get(Servo.class, "sE");
        this.sH = hardwareMap.get(Servo.class, "sH");;
    }

    /** THE CONSTRUCTOR GET THE MOTOR TO SPIN, POWER FOR THAT MOTOR, HARDWAREMAP, AND TELEMENTRY.  */
    public dip(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sD = hardwareMap.get(Servo.class, "sE");
        this.sH = hardwareMap.get(Servo.class, "sH");;
        this.telemetry = telemetry;
    }

    // spin Intake motor with power.
    public void getFreight() throws InterruptedException {
        sD.setPosition(intakePosition);
        sH.setPosition(holdingPosition);
        Thread.sleep(500);
    }
    // spin Intake motor with minos power.
    public void releaseFreightPos() throws InterruptedException {
        sD.setPosition(releasingPosition);
        sH.setPosition(holdingPosition);
        Thread.sleep(500);
    }

    public void releaseFreight() throws InterruptedException {
        sH.setPosition(pushingPosition);
        Thread.sleep(500);
    }

    // display power of motor.
    public void displayTelemetry(){
        telemetry.addLine("Servo freight at: " + sD.getPosition());
        telemetry.update();
    }

}
