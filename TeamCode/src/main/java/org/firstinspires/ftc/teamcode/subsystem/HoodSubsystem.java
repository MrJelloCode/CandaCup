package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.constants.TeleOpConstants;

public class HoodSubsystem {

    private Servo hood;

    public HoodSubsystem(HardwareMap hw) {
        hood = hw.get(
                Servo.class,
                TeleOpConstants.Hood.HOOD_SERVO_NAME
        );
    }

    public void setPosition(double pos){
        hood.setPosition(pos);
    }

    public double getPosition(){
        return hood.getPosition();
    }

    public void update(
            Pose robotPose,
            double targetX,
            double targetY
    ){

        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        double distance = Math.hypot(dx, dy);

        double hoodPos = interpolate(distance);

        hood.setPosition(hoodPos);
    }

    private double interpolate(double distance){

        double[][] table =
                TeleOpConstants.Hood.LOOKUP_TABLE;

        if(distance <= table[0][0]){
            return table[0][1];
        }

        if(distance >= table[table.length-1][0]){
            return table[table.length-1][1];
        }

        for(int i=0;i<table.length-1;i++){

            double d1 = table[i][0];
            double p1 = table[i][1];

            double d2 = table[i+1][0];
            double p2 = table[i+1][1];

            if(distance >= d1 && distance <= d2){

                double t =
                        (distance-d1)/(d2-d1);

                return p1 + t*(p2-p1);
            }
        }

        return table[table.length-1][1];
    }
}