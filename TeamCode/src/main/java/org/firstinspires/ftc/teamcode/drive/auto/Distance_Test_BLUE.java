package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
//@Disabled
public class Distance_Test_BLUE extends OpMode{
    DistanceSensor distance2;
    DistanceSensor distance4;

    int teamProp;

    @Override
    public void init() {
        distance1 = hardwareMap.get(DistanceSensor.class, "distance2");
        distance3 = hardwareMap.get(DistanceSensor.class, "distance4");


    }

    @Override
    public void init_loop() {
        if (distance2.getDistance(DistanceUnit.CM)<200) {
            teamProp = 3;
        }
        else if (distance4.getDistance(DistanceUnit.CM)<200) {
            teamProp = 2;
        }
        else {
            teamProp = 1;
        }
        telemetry.addData("Prop Place: ", teamProp);
        telemetry.addData("Distance1: ", distance2.getDistance(DistanceUnit.CM));
        telemetry.addData("Distance2: ", distance4.getDistance(DistanceUnit.CM));
    }


    @Override
    public void loop() {


    }
}




