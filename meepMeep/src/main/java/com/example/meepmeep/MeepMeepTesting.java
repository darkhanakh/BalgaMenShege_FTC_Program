package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive ->

                                //drive.trajectorySequenceBuilder(new Pose2d(-65.1, 36, 0))
//                                //.splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(90)), 0)
                                //.splineToSplineHeading(new Pose2d())
//                                .splineToConstantHeading(new Vector2d(0, 28), 45)
//                                //.splineToSplineHeading(new Pose2d(-12, 48, Math.toRadians(135)), 0)
//                                //.splineToSplineHeading(new Pose2d(-3.5,25.5,Math.toRadians(135)),Math.toRadians(-45))
//                                .waitSeconds(1)
//                                .back(0.01)
//                                .splineToSplineHeading(new Pose2d(-15, 48, Math.toRadians(-90)), Math.toRadians(90))
//                                //.splineToSplineHeading(new Pose2d(-15,46, Math.toRadians(0)), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(-15, 64), Math.toRadians(90))

//
//                                .splineToSplineHeading(new Pose2d(-36, 36, 0), 0)
//                                .splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(45)), 0)
//                                .splineToConstantHeading(new Vector2d(-3.0,27), Math.toRadians(-45))
//                                //start to junction
//                                .splineToSplineHeading(new Pose2d(-12, 48, 0), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(-12, 57), Math.toRadians(90))
//                                //.waitSeconds(0.8)
//                                //junction to stack
//                                .splineToConstantHeading(new Vector2d(-12, 48), Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(-3, 27, Math.toRadians(-45)), Math.toRadians(-45))
//                                //.waitSeconds(0.8)
//                                //stack to junction
//                                .splineToSplineHeading(new Pose2d(-12, 48, 0), Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(-12, 57), Math.toRadians(90))
//                                //.waitSeconds(0.8)
//                                //junction to stack 2
//                                .splineToConstantHeading(new Vector2d(-12, 48), Math.toRadians(-90))
//                                .splineToSplineHeading(new Pose2d(-3, 27, Math.toRadians(-45)), Math.toRadians(-45))
//                                .build()


                                //.splineToConstantHeading(new Vector2d(-12, 36), Math.toRadians(0))
//                                .splineToConstantHeading(new Vector2d(0, 28), Math.toRadians(-90))
//

                                //start to junction
                                drive.trajectorySequenceBuilder(new Pose2d(-65.1, 36, 0))
                                            .splineToConstantHeading(new Vector2d(-36, 36), 0)
                                .splineToSplineHeading(new Pose2d(-12, 36, Math.toRadians(45)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-2, 26), Math.toRadians(-45))                                        .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}