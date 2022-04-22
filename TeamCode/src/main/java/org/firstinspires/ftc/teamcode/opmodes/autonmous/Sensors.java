package org.firstinspires.ftc.teamcode.opmodes.autonmous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "sensors", group = "Autonomous sensors")
@Config

public class Sensors extends LinearOpMode {

    DistanceSensor intakeDistance = hardwareMap.get(DistanceSensor.class, "intakeDistance");
    double DistanceFromCarrierFreight;


    void getDistance(){

        DistanceFromCarrierFreight = intakeDistance.getDistance(DistanceUnit.CM);
        telemetry.addData("intakeDistance" , DistanceFromCarrierFreight);

    }

    public void runOpMode() throws InterruptedException {


        /* if(DistacneFromCarrierFreight > 20){

        ..........

        }








         */
    }
}
