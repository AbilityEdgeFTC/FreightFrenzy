package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class kufsa {

    /**
     *      HardwareMap:
     *      elevator = hardwareMap.get(DcMotor.class, "motorElevator");
     *      servo = hardwareMap.get(Servo.class, "sServo");
     *      CRservo = hardwareMap.get(CRServo.class, "sCRServo");
     */


    private Servo dor;/**kufsa dor servo**/
    private Servo kufsa; /**elevator servo*/

    private int kufsaPosition; /**kufsas Servo angle */
    private int dorPosition; /**kufsas dor Servo angle */

    private int minDorPos; /**Servo minimum angle*/
    private int maxDorPos; /**Servo maximum angle*/

    private int minKufsaPos; /**Servo minimum angle*/
    private int maxKufsaPos; /**Servo maximum angle*/


    private Telemetry telemetry;

    /**
     *Defines an object of an engine that is accepted for the engine used in this class
     */
    public kufsa(Servo kufsa,Servo dor,int maxDorPos,int minDorPos,int maxKufsaPos,int minKufsaPos) {
        this.kufsa = kufsa;
        this.dor = dor;
        this.maxDorPos = maxDorPos;
        this.minDorPos = minDorPos;
        this.maxKufsaPos = maxKufsaPos;
        this.minKufsaPos = maxKufsaPos;
    }

    public kufsa(Servo kufsa,Servo dor) {
        this.kufsa = kufsa;
        this.dor = dor;
    }



    public void OpenDor()  {
        dor.setPosition(maxDorPos);
    }
    public void CloseDor() {
        dor.setPosition(minKufsaPos);
    }
    public void KufsaUp() {
        kufsa.setPosition(maxKufsaPos);
    }
    public void KufsaDown() {
        kufsa.setPosition(minKufsaPos);
    }


/*

    /**
     *Gets power and time, and operates the engines on the power it received for the time it received.

    public void OpenDor(int maxDorPos)  {


        dor.setPosition(maxDorPos);

    }

    /**
     *Reaches the maximum angle of the servo we pre-defined in the constructor.

    public void CloseDor(int minKufsaPos) {
        dor.setPosition(minKufsaPos);
    }

    /**
     *Reaches the minimum angle of the servo we pre-defined in the constructor.

    public void KufsaUp(int maxKufsaPos) {
        kufsa.setPosition(maxKufsaPos);
    }

    public void KufsaDown(int minKufsaPos) {
        kufsa.setPosition(minKufsaPos);
    }

*/
    /**
     * telementry:
     *
     * @param minDorPosition
     * @param maxDorPosition
     * @param maxKufsaPosition
     * @param minKufsaPosition
     */
    public void Telemtry(boolean minDorPosition, boolean maxDorPosition,boolean minKufsaPosition,boolean maxKufsaPosition) {
        if(minDorPosition&&maxDorPosition)
            telemetry.addLine("Max dor position is power:"+maxDorPos+" Min dor position is power:"+minDorPos);
        else if (minDorPosition&&maxDorPosition)
            telemetry.addLine("Max kufsa position is:"+maxKufsaPos+"Min kufsa position is:"+minKufsaPos);
        else if (minDorPosition&&maxDorPosition&&minDorPosition&&maxDorPosition)
            telemetry.addLine("Max dor position is power:"+maxDorPos+" Min dor position is power:"+minDorPos+"Max kufsa position is:"+maxKufsaPos+"Min kufsa position is:"+minKufsaPos);

    }
}
