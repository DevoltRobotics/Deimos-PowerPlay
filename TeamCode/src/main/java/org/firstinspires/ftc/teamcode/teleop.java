package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp (name = "TeleOp", group = "teleops")
public class teleop extends OpMode {

    Robot robot = new Robot();

    boolean hombrolnto = false;
    boolean brazolnto = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap);
        robot.garra1(0);
        robot.garra2(0.6);
    }

    boolean yAnterior = false;
    boolean xAnterior = false;

    boolean hombroAutomatico = false;

    boolean elevAutomatico = false;
    double elevTarget = 0;

    @Override
    public void loop() {


        double trigger = gamepad1.left_trigger;

        if (gamepad1.right_trigger > 0.35) {
            trigger = gamepad1.right_trigger;
        }

        double turbo = 1 - (trigger * 0.65);


        telemetry.addData("odometria paralela", robot.drive.rightRear.getCurrentPosition());
        telemetry.addData("odometria perpendicular", robot.drive.rightFront.getCurrentPosition());
        telemetry.addData("corriente del hombro", robot.hombro.getCurrent(CurrentUnit.MILLIAMPS));
        //   telemetry.addData("velocidad del hombro", robot.hombro.getCurrent(Velocity));
        telemetry.addData("hombro", robot.hombro.getCurrentPosition());

        telemetry.addData("elev 1", robot.elev.getCurrentPosition());
        telemetry.addData("elev target", robot.rielesController.getTargetPosition());
        telemetry.addData("elev 2", robot.elev2.getCurrentPosition());

        telemetry.addData("brazo",robot.brazo.getCurrentPosition());
        telemetry.addData("hombro", robot.hombro.getCurrentPosition());


        if (gamepad2.right_bumper){
            hombrolnto = true;
        }else{
            hombrolnto = false;
        }

        if (gamepad2.left_bumper){
            brazolnto = true;
        }else {
            brazolnto = false;
        }




        if (brazolnto) {
            robot.brazito(-gamepad2.right_stick_y * 0.3);
        }else {
            robot.brazito(-gamepad2.right_stick_y);
        }


        if(gamepad2.y && gamepad2.y != yAnterior) {
            // se ejecuta una sola vez cuando presionas Y

            hombroAutomatico    = true;
            robot.hombritoAuto(0.7,0);
        } else if(gamepad2.x && gamepad2.x != xAnterior) {
            // se ejecuta una sola vez cuando presionas X

            //hombroAutomatico  = true;
            //robot.brazoauto(1,0);
        } else if(!hombroAutomatico|| Math.abs(gamepad2.right_trigger) >= 0.3 || Math.abs(gamepad2.left_trigger) >= 0.3) {
            hombroAutomatico = false;

            robot.hombro.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            if (hombrolnto) {
                robot.hombro.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * 0.3);
            } else {
                robot.hombro.setPower((gamepad2.left_trigger - gamepad2.right_trigger) * 0.7);
            }
        }
       /*    if (gamepad2.dpad_right){
               robot.garra1(robot.garra1.getPosition() + 0.1);
           }    else if (gamepad2.dpad_left){
               robot.garra1(robot.garra1.getPosition() - 0.1);
           }*/

       if (gamepad2.a) {
           robot.garra1(0.5);
           robot.garra2(0);
        } else if (gamepad2.b) {
           robot.garra1(0.2);
           robot.garra2(0.4);
       }


       if(Math.abs(gamepad2.left_stick_y) > 0.4) {
           elevTarget -= gamepad2.left_stick_y * 60;
        } else {
            if(gamepad2.dpad_up) {
                elevTarget = 1500;
            }else if (gamepad2.dpad_down){
                elevTarget = 0;
            }
        }

       elevTarget = Range.clip(elevTarget, 0, 1500);

        robot.elevadores(1, (int) elevTarget);

        robot.drive.update();

        //Vector2d input = new Vector2d(-gamepad1.left_stick_y * turbo,-gamepad1.left_stick_x * turbo).rotated(-robot.drive.getPoseEstimate().getHeading());

       if (gamepad1.dpad_up){
           robot.drive.setWeightedDrivePower(new Pose2d(1 * turbo, 0, 0));
       }else if (gamepad1.dpad_down){
           robot.drive.setWeightedDrivePower(new Pose2d(-1 * turbo,0,0));
       } else if (gamepad1.dpad_right){
           robot.drive.setWeightedDrivePower(new Pose2d(0,1 * turbo,0));

       }else if (gamepad1.dpad_left){
           robot.drive.setWeightedDrivePower(new Pose2d(0,-1 * turbo,0));
       }
       else {
           robot.drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y * turbo, -gamepad1.left_stick_x * turbo, -gamepad1.right_stick_x * turbo));
       }
        if (gamepad1.y){
            robot.hombritoAuto(1, 0);
        }

        yAnterior = gamepad2.y;
        xAnterior = gamepad2.x;
    }
}
