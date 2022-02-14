package org.firstinspires.ftc.teamcode.robot.subsystems.menu;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class taskObject {
    private static String[] options = {};
    private static String name;
    private static trajectoryObject trajectory;
    Pose2d pose2d;

    public taskObject(String[] options, String name) {
        this.options = options;
        this.name = name;
    }

    public taskObject(String[] options, String name, Pose2d[] points) {
        this.options = options;
        this.name = name;
        this.trajectory = trajectory;
    }

    public static void addOption(String newOprtion) {
        taskObject.options[options.length+1] = newOprtion;
    }

    public static String getOption(int place) {
        return options[place];
    }

    public static int size() {
        return options.length;
    }

    public static String getName() {
        return name;
    }

    public static void setName(String name) {
        taskObject.name = name;
    }
}
