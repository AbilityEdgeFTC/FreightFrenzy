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


import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robot.subsystems.cGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystems.carusela;



@Config
@TeleOp(name = "colorSensor Testing LOCALIZER", group = "testing")
public class colorSensorLocalizerDemo extends LinearOpMode {
    double y_cord = 30;

    public static int minRed = 200, minGreen = 200, minBlue = 200;
    public static int maxRed = 255, maxGreen = 255, maxBlue = 255;

    double oldYCords = y_cord;
    ColorSensor sensor;

    float[] hsv = {0, 0, 0};

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        cGamepad gamepad = new cGamepad(gamepad1);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()) {

            if (inMinMax(minRed, maxRed, sensor.red()) && inMinMax(minGreen, maxGreen, sensor.green()) && inMinMax(minBlue, maxBlue, sensor.blue())) {
                y_cord = 10;
            }

            if (gamepad.YOnce()) {
                y_cord = oldYCords;
            }

            telemetry.addData("Alpha: ", sensor.alpha());
            telemetry.addData("Red:  ", sensor.red());
            telemetry.addData("Green:  ", sensor.green());
            telemetry.addData("Blue: ", sensor.blue());
            Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsv);
            telemetry.addData("Red: ", hsv[0]);
            telemetry.addData("Green:  ", hsv[1]);
            telemetry.addData("Blue: ", hsv[2]);
            telemetry.update();
        }


    }

    boolean inMinMax(int min, int max, int value)
    {
        return min <= value && value <= max;
    }
}