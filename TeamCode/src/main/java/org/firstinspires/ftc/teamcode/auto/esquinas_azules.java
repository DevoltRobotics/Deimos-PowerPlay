package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.apache.commons.math3.stat.descriptive.UnivariateStatistic;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


@Autonomous(name = "esquinas azules")
public class esquinas_azules extends LinearOpMode {

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

        int position = 2;

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();

            if (detections.size() >= 1) {
                position = detections.get(0).id;
                telemetry.addData("id detectada", position);
                telemetry.update();
            }
        }


        Robot robot = new Robot();
        robot.init(hardwareMap);

        Pose2d posicionInicial = new Pose2d(-36, 60, Math.toRadians(270));

        TrajectorySequenceBuilder sequenceee = robot.drive.trajectorySequenceBuilder(posicionInicial)
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> {
                    robot.garra2(0);
                    robot.garra1(0.5);
                })

                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.brazoauto(1, 700);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.hombritoAuto(1, 400);
                    robot.elevadores(1, 1500);

                })
                /*.UNSTABLE_addTemporalMarkerOffset(2.5, ()->{
                    robot.elevadorAuto(1, -2500);
                })*/
                /* .UNSTABLE_addTemporalMarkerOffset(1.5, ()->{
                     robot.brazoauto(1,970);
                 })*/

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.garra1(0.2);
                    robot.garra2(0.4);
                })
                .lineTo(new Vector2d(-36, -3))
                .waitSeconds(1)
                .setReversed(true)

                .lineToConstantHeading(new Vector2d(-35, 9))
                .splineToLinearHeading(new Pose2d(-36, 9, Math.toRadians(180)), 0);

        int ciclos = 2;
        int[] elevadorPosition = {800};

        for (int i = 1; i <= ciclos; i++) {
            sequenceee
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {

                    })
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        robot.hombritoAuto(0.5, 0);
                        System.out.println("que ooonda saray");
                    })

                    .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                        robot.brazoauto(1, 0);
                        System.out.println("que ooonda abby");
                        robot.elevadores(1, elevadorPosition[0]);
                    })

                    .UNSTABLE_addTemporalMarkerOffset(1.15, () -> {
                        robot.garra1(0.5);
                        robot.garra2(0);
                    })

                    .lineToConstantHeading(new Vector2d(-61.3, 10)) // TODO: Posicion para agarrar

                    // .splineToLinearHeading(new Pose2d(-40,11, Math.toRadians(180)), Math.toRadians(0))

                    .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                        robot.elevadores(1, 1500);
                    })

                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        robot.hombritoAuto(1, 400);
                        robot.brazoauto(1, 700);
                    })

                    .lineToConstantHeading(new Vector2d(-24.7, 10), // TODO: Posicion para poner
                            SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL * 0.5) // aceleracion
                    )
                    .lineToConstantHeading(new Vector2d(-24.7, 6.15)) // acercarse

                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        robot.garra1(0.2);
                        robot.garra2(0.4);
                    })

                    .waitSeconds(0.6);

            elevadorPosition[0] -= 80;
        }

        TrajectorySequence autonomoPrePark = sequenceee.build();

        waitForStart();

        TrajectorySequenceBuilder parkSequenceBuildetr = robot.drive.trajectorySequenceBuilder(autonomoPrePark.end());

        if (position == 2) {
            parkSequenceBuildetr.lineToConstantHeading(new Vector2d(-35, 8.8));
        } else if (position == 1) {
            parkSequenceBuildetr.lineToConstantHeading(new Vector2d(-36, 12)); //izq,der/front,back primer cono
            parkSequenceBuildetr.lineToConstantHeading(new Vector2d(-10, 9.1));
            parkSequenceBuildetr.lineToConstantHeading(new Vector2d(-10, 32));
        } else if (position == 3) {
            parkSequenceBuildetr.lineToConstantHeading(new Vector2d(-36, 7.1));
            parkSequenceBuildetr.lineToConstantHeading(new Vector2d(-60, 7.1));
            parkSequenceBuildetr.lineToConstantHeading(new Vector2d(-60, 30));
        }

        TrajectorySequence parkSequence = parkSequenceBuildetr.build();

        robot.drive.setPoseEstimate(posicionInicial);
        robot.drive.followTrajectorySequence(autonomoPrePark);
        robot.drive.followTrajectorySequence(parkSequence);
    }
}
