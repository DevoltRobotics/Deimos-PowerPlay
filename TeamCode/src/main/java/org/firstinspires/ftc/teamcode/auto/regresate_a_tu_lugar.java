package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Disabled
@Autonomous
public class regresate_a_tu_lugar extends LinearOpMode {

    int position = 1;
@Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot();
        robot.init(hardwareMap);


        Pose2d posicionInicial = new Pose2d(-10,15, Math.toRadians(270));

        TrajectorySequenceBuilder sequenceBuilder = robot.drive.trajectorySequenceBuilder(posicionInicial)
                .lineTo(new Vector2d(-34, 1))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-49, 12.4), Math.toRadians(0.0))
                .lineTo(new Vector2d(-56, 12.4));

       /* if(position == 0) {
            sequenceBuilder.lineTo();
        }*/

        TrajectorySequence sequence = sequenceBuilder.build();

        TrajectorySequence sequence2 = robot.drive.trajectorySequenceBuilder(posicionInicial)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-36, 60),Math.toRadians(180))
                .build();

        TrajectorySequenceBuilder sequenceBuilder3 = robot.drive.trajectorySequenceBuilder(posicionInicial)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () ->{
                    robot.brazito(0.7);
                    robot.elevadorAuto(1,-2900);
                    robot.hombrito(-0.5);
                })

                .UNSTABLE_addTemporalMarkerOffset(4, () ->{
                    robot.garra1(0.7);
                    robot.garra2(0.3);
                })
                .lineTo(new Vector2d(-33, -5))
                .waitSeconds(5)
                .setReversed(true);


        if(position == 2){
            sequenceBuilder3.lineToConstantHeading(new Vector2d(-37,38));
        }else if (position == 1){
            sequenceBuilder3.splineToConstantHeading(new Vector2d(-10,15),Math.toRadians(0.0));
        }else if (position == 3){
            sequenceBuilder3.splineToConstantHeading(new Vector2d(-60,15),Math.toRadians(0.0));
        }

        waitForStart();

        robot.drive.setPoseEstimate(posicionInicial);

        robot.drive.followTrajectorySequence(sequence2);

    }
}
