/**
 * Created by Ability Edge#18273
 * - Elior Yousefi
 */

package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class hand {

    Servo sH;
    public static double intakePos = 0;
    public static double level1Hub = .9, level2Hub = .735, level3Hub = .5, levelSharedHub = .8, levelDuck3 = 0.6, levelDuck2 = .77, levelDuck1 = 1;

    public enum HandPos
    {
        INTAKE,
        SHARED_HUB,
        ONE_HUB,
        TWO_HUB,
        THREE_HUB,
        DUCK1,
        DUCK2,
        DUCK3
    }

    public HandPos handPos = HandPos.INTAKE;

    public hand(HardwareMap hardwareMap) {
        this.sH = hardwareMap.get(Servo.class, "sH");
        this.sH.setDirection(Servo.Direction.REVERSE);
    }

    public void intake()
    {
        handPos = HandPos.INTAKE;
        sH.setPosition(intakePos);
    }

    public void shared()
    {
        handPos = HandPos.SHARED_HUB;
        sH.setPosition(levelSharedHub);
    }

    public void level1()
    {
        handPos = HandPos.ONE_HUB;
        sH.setPosition(level1Hub);
    }

    public void level2()
    {
        handPos = HandPos.TWO_HUB;
        sH.setPosition(level2Hub);
    }

    public void level3()
    {
        handPos = HandPos.THREE_HUB;
        sH.setPosition(level3Hub);
    }

    public void moveTo(double position)
    {
        sH.setPosition(position);
    }

    public void level3Duck()
    {
        handPos = HandPos.DUCK3;
        sH.setPosition(levelDuck3);
    }

    public void level2Duck()
    {
        handPos = HandPos.DUCK2;
        sH.setPosition(levelDuck2);
    }

    public void level1Duck()
    {
        handPos = HandPos.DUCK1;
        sH.setPosition(levelDuck1);
    }

    public HandPos getHandPos() {
        return handPos;
    }

    public double getPos() {
        return sH.getPosition();
    }


    public void setHandPos(HandPos handPos) {
        this.handPos = handPos;
    }

    public static double getIntakePos() {
        return intakePos;
    }

    public static double getLevel1Hub() {
        return level1Hub;
    }

    public static double getLevel2Hub() {
        return level2Hub;
    }

    public static double getLevel3Hub() {
        return level3Hub;
    }

    public static double getLevelSharedHub() {
        return levelSharedHub;
    }

    public static double getLevelDuck3() {
        return levelDuck3;
    }

    public static double getLevelDuck2() {
        return levelDuck2;
    }

    public static double getLevelDuck1() {
        return levelDuck1;
    }

    public static void setIntakePos(double intakePos) {
        hand.intakePos = intakePos;
    }

    public static void setLevel1Hub(double level1Hub) {
        hand.level1Hub = level1Hub;
    }

    public static void setLevel2Hub(double level2Hub) {
        hand.level2Hub = level2Hub;
    }

    public static void setLevel3Hub(double level3Hub) {
        hand.level3Hub = level3Hub;
    }

    public static void setLevelSharedHub(double levelSharedHub) {
        hand.levelSharedHub = levelSharedHub;
    }

    public static void setLevelDuck3(double levelDuck3) {
        hand.levelDuck3 = levelDuck3;
    }

    public static void setLevelDuck2(double levelDuck2) {
        hand.levelDuck2 = levelDuck2;
    }

    public static void setLevelDuck1(double levelDuck1) {
        hand.levelDuck1 = levelDuck1;
    }
}
