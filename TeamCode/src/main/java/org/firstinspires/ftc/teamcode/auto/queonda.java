package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


@Autonomous(name = "ezquinas rojas")
public class queonda extends LinearOpMode {



    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;


    double tagsize = 0.166;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);


        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(pipeline);


        camera.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        int position = 0;

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

            if (detections.size() >= 1) {
                position = detections.get(0).id;
                telemetry.addData("id detectada", position);

            }
        }


        Robot robot = new Robot();
        robot.init(hardwareMap);




        Pose2d posicionInicial = new Pose2d(36, 60, Math.toRadians(270));

        TrajectorySequenceBuilder sequenceBuilder = robot.drive.trajectorySequenceBuilder(posicionInicial)
                .lineTo(new Vector2d(-34, 1))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-49, 12.4), Math.toRadians(0.0))
                .lineTo(new Vector2d(-56, 12.4));

       /* if(position == 0) {
            sequenceBuilder.lineTo();
        }*/

        TrajectorySequence sequence = sequenceBuilder.build();

        TrajectorySequenceBuilder sequenceee = robot.drive.trajectorySequenceBuilder(posicionInicial)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    robot.garra2(0);
                    robot.garra1(1);
                })


                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () ->{
                    robot.brazoauto(1,550);
                    robot.elevadorAuto(1,-3500);
                    robot.hombritoAuto(1,300);
                })
                .UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    robot.elevadorAuto(1, -2500);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () ->{
                    robot.garra1(0);
                    robot.garra2(1);
                })
                .lineTo(new Vector2d(36.4, -3.95))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () ->{
                    robot.elevadorAuto(1,0);
                    System.out.println("que ooonda santiago");
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () ->{
                    robot.brazoauto(1,220);
                    System.out.println("que ooonda abby");
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    robot.hombritoAuto(0.5,0);
                    System.out.println("que ooonda saray");
                })
                .setReversed(true);


        if(position == 2){
            sequenceee.lineToConstantHeading(new Vector2d(36,7.1));
            sequenceee.lineToConstantHeading(new Vector2d(37,25));
        }else if (position == 1){
            sequenceee.lineToConstantHeading(new Vector2d(35,8.1));
            sequenceee.lineToConstantHeading(new Vector2d(10,7.1));
            sequenceee.lineToConstantHeading(new Vector2d(10,25));
        }else if (position == 3){
            sequenceee.lineToConstantHeading(new Vector2d(36,7.1));
            sequenceee.lineToConstantHeading(new Vector2d(60,7.1));
            sequenceee.lineToConstantHeading(new Vector2d(60,25));
        }

        TrajectorySequenceBuilder sequence2 = robot.drive.trajectorySequenceBuilder(posicionInicial)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () ->{
                    robot.brazoauto(1,550);
                    robot.elevadorAuto(1,-3500);
                    robot.hombritoAuto(1,300);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    robot.elevadorAuto(1, -2500);
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () ->{
                    robot.garra1(0);
                    robot.garra2(1);
                })
                .lineTo(new Vector2d(36, -5))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () ->{
                    robot.elevadorAuto(1,0);
                    System.out.println("que ooonda santiago");
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () ->{
                    robot.brazoauto(1,220);
                    System.out.println("que ooonda abby");
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    robot.hombritoAuto(0.5,0);
                    System.out.println("que ooonda saray");
                })
                .setReversed(true)

                .lineToConstantHeading(new Vector2d(35,11))
                .splineToLinearHeading(new Pose2d(40,11,Math.toRadians(0)), 0)

                // .splineToLinearHeading(new Pose2d(-40,11, Math.toRadians(180)), Math.toRadians(0))

                .UNSTABLE_addTemporalMarkerOffset(2,() ->{
                    robot.garra1(1);
                    robot.garra2(0);
                })


                .lineToConstantHeading(new Vector2d(59,10))

                .UNSTABLE_addTemporalMarkerOffset(2,() ->{
                    robot.elevadorAuto(1,-3500);
                })


                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(25,19))
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    robot.brazoauto(1,550);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    robot.hombritoAuto(1,300);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, ()->{
                    robot.garra1(0);
                    robot.garra2(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    robot.elevadorAuto(1,0);
                    System.out.println("que ooonda santiago");
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () ->{
                    robot.brazoauto(1,220);
                    System.out.println("que ooonda abby");
                })
                .UNSTABLE_addTemporalMarkerOffset(0,() ->{
                    robot.hombritoAuto(0.5,0);
                    System.out.println("que ooonda saray");
                });
                /* .lineTo(new Vector2d(-56,12))
                 .lineTo(new Vector2d(-23,12))
                 .lineToConstantHeading(new Vector2d(-35,12))
                 .lineToConstantHeading(new Vector2d(-35,24))*/
        if(position == 2){
            sequence2.lineToConstantHeading(new Vector2d(37,19));
            sequence2.lineToLinearHeading(new Pose2d(37,38,90));
        }else if (position == 1){
            sequence2.lineToConstantHeading(new Vector2d(10,19));
            sequence2.splineToLinearHeading(new Pose2d(10,15,90),Math.toRadians(0.0));
        }else if (position == 3){
            sequence2.lineToConstantHeading(new Vector2d(60,19));
            sequence2.splineToLinearHeading(new Pose2d(60,15,90),Math.toRadians(0.0));
        }



        waitForStart();

        robot.drive.setPoseEstimate(posicionInicial);
        robot.drive.followTrajectorySequence(sequenceee.build());
        //  robot.drive.followTrajectorySequence(sequenceBuilder3.build());


        /*switch (jaaj) {
            case 1:
        robot.drive.followTrajectorySequence(sequence);
        break;
                 case 2:
             robot.drive.followTrajectorySequence(sequence2);
             break;
                    case 3:
                robot.drive.followTrajectorySequence(sequence3);
                break;*/


    }
}
