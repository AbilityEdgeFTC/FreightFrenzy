package org.firstinspires.ftc.teamcode.robot.roadrunner.localizers;

import android.content.Context;

 import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
 import org.firstinspires.ftc.ftccommon.external.OnDestroy;

 public class T265StartupHook {
     public static T265Camera slamera;

     @OnCreate
     public static void createSlamera(Context appContext) {
         Translation2d translation2d = new Translation2d(-0.065*39.37 , 0.14*39.37);
         Rotation2d rotation2d = new Rotation2d(Math.toRadians(90));
         slamera = new T265Camera(new Transform2d(translation2d, rotation2d), 0.5, appContext);
         slamera.start();
     }

     @OnDestroy
     public static void destroySlamera(Context appContext) {
         slamera.stop();
         slamera.free();
     }
 }