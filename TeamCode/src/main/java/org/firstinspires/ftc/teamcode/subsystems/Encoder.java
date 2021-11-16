package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Encoder {

    /**
     *      HardwareMap:
     *      encoder = hardwareMap.get(DcMotor.class, "motorEncoder");
     */


    /**
     * declaring motor as encoder , the encoder connects
     to a motor so we read the values from that motor.
     */
    private DcMotorEx encoder;

    /**
     * declaring telemetry for printing positions of the encoder, where its placed, and if its reversed or not.
     */
    private Telemetry telemetry;

    /** this is the position of the encoder, moving the encoder will update it with this int, its an int because the position
      from the encoder returns as an int.
     */
    private int position;


    /**
     * declaring if the encoder is in a reverse position.
      */
    private boolean reverse;

    /**
     * 2 constructors, each one gets the motor like we described up there! and a reverse boolean, also like we described.
     * the first one resets the position when you call the constructor, but the second one resets if you set a boolean to
       true or false. both of them get booleans of they are reversed or not.
     */
    public Encoder(DcMotorEx encoder, boolean reverse) {

        this.encoder = encoder;

        this.reverse = reverse;
        this.position = 0;

    }

    public Encoder(DcMotorEx encoder, boolean reverse, String placement, boolean reset) {

        this.encoder = encoder;

        this.reverse = reverse;

        if(reset){
            this.position = 0;
        }else{
            this.position = encoder.getCurrentPosition();
        }

    }

    /**
     * get and set the direction of an encoder, if to be reversed or not.
     * @return reverse - boolean
     */
    public Boolean getDirection() {
        return reverse;
    }

    public void setDirection(boolean reverse) {
        this.reverse = reverse;
    }


    /**
     * get and set the position of an encoder. if the encoder needs to be reversed by the boolean up the top, so we multiply the
       position of the encoder by -1.
     * @return encoder.getCurrentPosition() - int
     */
    public int getPosition(){
        if(this.reverse){
            return position * -1;
        }else{
            return position;
        }
    }

    public void setPosition(int position){
        this.position = position;
    }


    /**
     * @return telemntry for encoder placement, position value only, if reversed, or both.
     */
    public void toString(String placement, boolean position, boolean reversed) {
        if(position && reversed){
            telemetry.addData("Encoder", "placement (%.2f), position (%.2f)", placement, this.position);
            telemetry.addData("Encoder", "placement (%.2f), reversed (%.2f)", placement, this.reverse);
        }else if(position){
            telemetry.addData("Encoder", "placement (%.2f), position (%.2f)", placement, this.position);
        }else if(reversed){
            telemetry.addData("Encoder", "placement (%.2f), reversed (%.2f)", placement, this.reverse);
        }
    }
}
