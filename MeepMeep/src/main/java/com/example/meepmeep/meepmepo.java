package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class meepmepo {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, 60, Math.toRadians(270)))


                                .lineTo(new Vector2d(-33, -7))
                                .build()



                             /*   .lineTo(new Vector2d(-34, 1))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-56,12.2,Math.toRadians(180)), Math.toRadians(270))
                                .lineTo(new Vector2d(-23,12.4))
                                .lineTo(new Vector2d(-56,12.2))
                                .lineTo(new Vector2d(-23,12.4))
                                .lineToLinearHeading(new Pose2d(-35,12.4,Math.toRadians(270)))
                                .lineToConstantHeading(new Vector2d(-35,24))
                                .build()*/
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}