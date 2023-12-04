package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="auto_iterative_BLUE", group="auto")
@Disabled

public class auto_iterative_BLUE extends OpMode
{
    lift lift = new lift(hardwareMap);
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    DistanceSensor distance2;
    DistanceSensor distance4;


//THIS IS FOR BLUE SIDE

    int teamProp;

    //Middle of spike tapes
    public static double x1 = 18;
    public static double y1 = -2;
    //Middle of backdrop
    public static double x2 = 6;
    public static double y2 = 15;
    public static double x3 = 27;
    public static double y3 = 33;



    @Override
    public void init() {
        drive.setPoseEstimate(startPose);
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        distance4 = hardwareMap.get(DistanceSensor.class, "distance4");

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

    }



    @Override
    public void start() {

    }

    @Override
    public void loop() {
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
             //   .forward(10)

                // drive to spike marker tape, face it
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(teamProp)))

                //spit out pixel while backing up a bit
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        })
                .waitSeconds(1)
                .back(3)

                //back up to outside spike marks, face boardish
                .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(90)))

                //line to middle of board, face it
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(90)))


                //scoot left/right based on variable
                .strafeLeft(teamProp/10)


                //output other pixel
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {

                })
                //time to score
                .waitSeconds(3)

                //scoot right to park
                .strafeRight(10)

                .build();


        drive.update();

    }

    @Override
    public void stop() {

    }

}
