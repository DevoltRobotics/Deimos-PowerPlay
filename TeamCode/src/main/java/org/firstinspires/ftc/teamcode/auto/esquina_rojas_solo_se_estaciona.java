package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous (name = "esquina rojas solo se estaciona")
public class esquina_rojas_solo_se_estaciona extends LinearOpMode {

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

        TrajectorySequenceBuilder sequenceee = robot.drive.trajectorySequenceBuilder(posicionInicial)
                .waitSeconds(20);
        if(position == 2){
            sequenceee.lineToConstantHeading(new Vector2d(36,30));
        }else if (position == 1){
            sequenceee.lineToConstantHeading(new Vector2d(10,60));
            sequenceee.lineToConstantHeading(new Vector2d(10,30));
        }else if (position == 3){
            sequenceee.lineToConstantHeading(new Vector2d(60,60));
            sequenceee.lineToConstantHeading(new Vector2d(60,30));
        }

        waitForStart();

        robot.drive.setPoseEstimate(posicionInicial);
        robot.drive.followTrajectorySequence(sequenceee.build());

    }
}
