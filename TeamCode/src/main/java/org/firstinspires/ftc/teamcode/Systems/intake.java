package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class intake {

    /**
     *      HardwareMap:
     *      elevator = hardwareMap.get(DcMotor.class, "motorElevator");
     *      servo = hardwareMap.get(Servo.class, "sServo");
     *      CRservo = hardwareMap.get(CRServo.class, "sCRServo");
     */

    private DcMotor intake; /**elevator motor */

    private int power; /**Variable of motor power */
    private int autoPower; /**Variable of motor power for autonomus */
    private int autoTime; /**motor or servo operating time  for autonomus*/




    private Telemetry telemetry;

    /**
     *Defines an object of an engine that is accepted for the engine used in this class
     */
    public intake(DcMotor intake) {
        this.intake = intake;
    }




/**------------------------------------------------------------------------------------------------------------------------------------**/



    /**
     *Gets power and time, and operates the engines on the power it received for the time it received.
     */
    public void autoIntake(int autoPower, int autoTime) throws InterruptedException {

        intake.setPower(power);
        Thread.sleep(autoTime);
    }
    /**
     *Reaches the maximum angle of the servo we pre-defined in the constructor.
     */
    public void Intake(int power) {
        intake.setPower(power);
    }





    /**
     * telementry:
     *
     * @param Power
     * @param AutoPower
     * @param AutoTime
     */
    public void Telemtry(boolean Power, boolean AutoTime,boolean AutoPower) {
        if(Power)
        telemetry.addLine("intake power:"+power);
        else if (AutoTime&&AutoPower)
            telemetry.addLine("intake power:"+power+" for:"+autoPower);

    }
}
