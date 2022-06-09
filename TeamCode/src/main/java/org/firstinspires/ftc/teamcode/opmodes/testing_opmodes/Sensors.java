package org.firstinspires.ftc.teamcode.opmodes.testing_opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
@TeleOp(name = "Sensors")
@Disabled
public class Sensors extends LinearOpMode {

    double DistanceFromCarrierFreight;
    boolean InWarehouse;
    public static double distance = 10;

    ModernRoboticsI2cColorSensor WarehouseColorSensor = hardwareMap.get(ModernRoboticsI2cColorSensor.class ,"WarehouseColorSensor");
     DistanceSensor intakeDistance = hardwareMap.get(DistanceSensor.class, "intakeDistance");



     public void runOpMode() throws InterruptedException {
        while (opModeIsActive())
        {
            telemetry.addData("WarehouseDetected" , InWarehouse);
            telemetry.addData("ColorNumber" , WarehouseColorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
        }

        WarehouseColorSensor.enableLed(true);

        while (WarehouseColorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) != 16){

            if(WarehouseColorSensor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER) == 16){
                InWarehouse = true;

                //Put whatever the robot need to do while he has detected the color white
            }
            //Put whatever the robot need to do while he hasnt detected the color white

        }

        DistanceFromCarrierFreight = intakeDistance.getDistance(DistanceUnit.CM);
        telemetry.addData("intakeDistance" , DistanceFromCarrierFreight);

        if(DistanceFromCarrierFreight > distance){

            telemetry.addLine();
        }

        telemetry.update();
    }
}
