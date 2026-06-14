package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;

public class PathStorage {

    private static Pose storedPose = new Pose(0, 0, 0);

    public static void setPose(Pose pose) {
        storedPose = new Pose(
                pose.getX(),
                pose.getY(),
                pose.getHeading()
        );
    }

    public static Pose getPose() {
        return new Pose(
                storedPose.getX(),
                storedPose.getY(),
                storedPose.getHeading()-Math.toRadians(-4)
        );
    }

    public static void clear() {
        storedPose = new Pose(0, 0, 0);
    }
}