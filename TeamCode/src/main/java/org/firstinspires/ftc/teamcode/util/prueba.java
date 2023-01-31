package org.firstinspires.ftc.teamcode.util;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

public class prueba extends LinearOpMode {


    Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.elevadorAuto(0.5, 500);

    }
}
