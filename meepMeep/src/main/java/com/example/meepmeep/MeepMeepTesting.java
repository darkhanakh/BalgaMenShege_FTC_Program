package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d startPose = new Pose2d(-65.1, -36, Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(startPose)
                                        .lineToConstantHeading(new Vector2d(-3, -36))
                                        .lineToConstantHeading(new Vector2d(-3, -26.5))
                                        .lineToConstantHeading(new Vector2d(-3 , -36))
                                        .lineToConstantHeading(new Vector2d(-12, -36))
                                        .lineToConstantHeading(new Vector2d(-12, -66.5))
                                        .lineToConstantHeading(new Vector2d(-13, -44))
                                        .splineTo(new Vector2d(-1, -25), Math.toRadians(-315))
                                        .setTangent(Math.toRadians(-135))
                                        .splineTo(new Vector2d(-12, -48), Math.toRadians(-90))
//                                      .lineToSplineHeading(new Pose2d(-12, -12, Math.toRadians(0)))
                                        .lineToSplineHeading(new Pose2d(-12, -34, Math.toRadians(0)))
//                                      .lineToSplineHeading(new Pose2d(-12, -60, Math.toRadians(0)))
                                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}