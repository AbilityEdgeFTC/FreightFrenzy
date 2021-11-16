package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.time.Year;

class Elevator {

    /**
     *      HardwareMap:
     *      elevator = hardwareMap.get(DcMotor.class, "motorElevator");
     *      servo = hardwareMap.get(Servo.class, "sServo");
     *      CRservo = hardwareMap.get(CRServo.class, "sCRServo");
     */

    private DcMotor arm; /**elevator motor */
    private Servo armServo; /**elevator servo*/
    private CRServo armCRServo; /**elevator CRservo*/

    private int power; /**Variable of motor power */
    private int time; /**motor or servo operating time */
    private int position; /**Servo angle */

    private int MinPos; /**Servo minimum angle*/
    private int MaxPos; /**Servo maximum angle*/

    private int CRpower; /**Power of CRservo*/

    private Telemetry telemetry;

    /**
     *Defines an object of an engine that is accepted for the engine used in this class
     */
    public Elevator(DcMotor elevator) {
        this.arm = elevator;
    }

    /**
     *Defines a motor and servo object obtained from a servo and motor used in this class
     */

    public Elevator(Servo armServo,DcMotor elevator ) {
        this.armServo = armServo;
        this.arm = elevator;
    }

    /**
     *Defines a motor and CRServo  object obtained from a CRServo and motor used in this class
     */
    public Elevator(CRServo armCRServo,DcMotor elevator ) {
        this.armCRServo = armCRServo;
        this.arm = elevator;
    }

    /**
     *Defines a received servo and CRServo object for a servo and CRServo used in this class
     */
    public Elevator(Servo armServo, CRServo armCRServo) {
        this.armServo = armServo;
        this.armCRServo = armCRServo;
    }

    /**
     *Defines a received servo and CRServo and engine object for a servo and CRServo and engine used in this class
     */
    public Elevator(DcMotor arm, Servo armServo, CRServo armCRServo) {
        this.arm = arm;
        this.armServo = armServo;
        this.armCRServo = armCRServo;
    }

    /**
     *Sets the minimum and maximum angle of the servo to the minimum and maximum in this class
     */
    public Elevator(int minPos, int maxPos) {
      this.MinPos = minPos;
      this.MaxPos = maxPos;
    }




    /**
     *Gets power and time, and operates the engines on the power it received for the time it received.
     */
    public void TimePower(int power, int time) throws InterruptedException {
        this.power = power;
        this.time = time;

        arm.setPower(power);
        Thread.sleep(time);
    }

    /**
     *Reaches the maximum angle of the servo we pre-defined in the constructor.
     */
    public void MaxPos(int MaxPoss) {
        armServo.setPosition(MaxPos);
    }

    /**
     *Reaches the minimum angle of the servo we pre-defined in the constructor.
     */
    public void MinPos(int MinPoss) {
        armServo.setPosition(MinPos);
    }

    /**
     *Gets an angle, reaches the same angle we got in the servo.
     */
    public void Pos(int position) {
        this.position = position;

        armServo.setPosition(position);
    }

    /**
     *Gets power, and exerts the same power on the CRServo.
     */
    public void armCRServo(int power) {
       this.power = power;

        armCRServo.setPower(power);
    }

    /**
     *  Gets power and time, and exerts the same power on the CRServo for the time we get.
     */
    public void armCRServoT(int CRpower,int time) throws InterruptedException {
        this.CRpower = power;
        this.time = time;

        armCRServo.setPower(power);
        Thread.sleep(time);
    }

    /**
     * telementry:
     *
     * @param power
     * @param position
     * @param CRpower
     */
    public void Telemtry(boolean power, boolean position,boolean CRpower) {
        if(power&&position&&CRpower)
        telemetry.addLine("arm power:"+power+"position:"+position+"CRpower:"+CRpower);
        else if (power&&position)
            telemetry.addLine("arm power:"+power+"position:"+position);
        else if (CRpower&&position)
            telemetry.addLine("CRpower:"+CRpower+"position:"+position);
        else if (CRpower&&power)
            telemetry.addLine("arm power:"+power+"CRpower:"+CRpower);
        else if (power)
            telemetry.addLine("arm power:"+power);
        else if (position)
            telemetry.addLine("position:"+position);
        else if (CRpower)
            telemetry.addLine("CRpower:"+CRpower);
    }
}
