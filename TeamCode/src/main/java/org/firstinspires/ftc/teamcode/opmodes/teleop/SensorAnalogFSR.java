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

package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.AnalogSensor;

import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.threads.IntakeFixingThread;

@Config
@TeleOp(group = "Sensor")
public class SensorAnalogFSR extends LinearOpMode {

    //AnalogSensor freightSensor;  // Hardware Device Object
    AnalogInput freightSensor;
    public static double voltageThreshold = (float) 0.00000000000000001;
    IntakeFixingThread intakeFixingThread;
    @Override
    public void runOpMode() {

        freightSensor = hardwareMap.get(AnalogInput.class, "freightSensor");
        intake intake = new intake(hardwareMap);
        intakeFixingThread = new IntakeFixingThread(hardwareMap, telemetry);
        //analogInputController = hardwareMap.get(AnalogInputController.class, "freightSensor");
        //analogInput = new AnalogInput(analogInputController, 0);

        intakeFixingThread.start();

        waitForStart();

        while (opModeIsActive()) {

            if(freightSensor.getVoltage() > voltageThreshold){
                intake.intakeForward();
            } else {
                intake.stop();
            }

            telemetry.addData("Sensor Value", freightSensor.getVoltage());
            telemetry.update();
        }

        intakeFixingThread.exitThread();
        intakeFixingThread.interrupt();
    }
}
