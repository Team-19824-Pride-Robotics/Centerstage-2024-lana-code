package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
//@Disabled
public class Distance_Test extends OpMode{
    DistanceSensor distance1;
    DistanceSensor distance2;

    int teamProp;

    @Override
    public void init() {
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");

    }

    @Override
    public void init_loop() {
        if (distance1.getDistance(DistanceUnit.CM)<200) {
            teamProp = 1; }
        else if (distance2.getDistance(DistanceUnit.CM)<200) {
            teamProp = 2; }
        else {
            teamProp = 3;}
        telemetry.addData("Prop Place: ", teamProp);
        telemetry.addData("Distance1: ", distance1.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance2: ", distance2.getDistance(DistanceUnit.CM));
    }


    @Override
    public void loop() {


    }
}




