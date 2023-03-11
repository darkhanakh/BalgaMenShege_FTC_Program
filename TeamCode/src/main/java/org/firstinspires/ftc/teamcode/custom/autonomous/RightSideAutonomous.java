package org.firstinspires.ftc.teamcode.custom.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.AprilTagsDetector;


@Autonomous(name = "КОНСИСТЕНТ ПАРКИНГ")
public class RightSideAutonomous extends LinearOpMode {
    private Servo claw = null;
    private CRServo lift = null;
    private Servo tower = null;
    private AprilTagsDetector tagsDetector;
    private AprilTagsDetector.Tag foundTag;

    double voltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }


    @Override
    public void runOpMode() {
        tagsDetector = new AprilTagsDetector(hardwareMap, telemetry);
        foundTag = AprilTagsDetector.Tag.noTag;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-65.1, -36, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        claw = hardwareMap.get(Servo.class, "claw");
        tower = hardwareMap.get(Servo.class, "tower");
        lift = hardwareMap.get(CRServo.class, "lift");

        while (opModeInInit()) {
            if (opModeInInit()) {
                claw.setPosition(0.5);
                claw.setPosition(0.5);
            }
            AprilTagsDetector.Tag tempTag = tagsDetector.getTag();
            if (tempTag != AprilTagsDetector.Tag.noTag) {
                foundTag = tempTag;
                telemetry.addData("tag", foundTag);
            }
        }



        if (isStopRequested()) return;

        double kVol = 13 / voltage();

        telemetry.addLine("trajectories building...");
        telemetry.update();


        TrajectorySequence leftParking = drive.trajectorySequenceBuilder(startPose)
                .forward(53)
                .strafeLeft(27)
                .build();

        TrajectorySequence midParking = drive.trajectorySequenceBuilder(startPose)
                .forward(53)
                .build();

        TrajectorySequence rightParking = drive.trajectorySequenceBuilder(startPose)
                .forward(53)
                .strafeRight(27)
                .build();

        TrajectorySequence junctionParkSetup = drive.trajectorySequenceBuilder(new Pose2d(-12, -36, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setLift(-0.4))
                .UNSTABLE_addTemporalMarkerOffset(0,() -> setTower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> setTower(0))
                .lineTo(new Vector2d(-36, -36))
                .build();

        TrajectorySequence jParkL = drive.trajectorySequenceBuilder(new Pose2d(-36, -36, 0))
                .lineTo(new Vector2d(-36, -12))
                .build();

        TrajectorySequence jParkM = drive.trajectorySequenceBuilder(new Pose2d(-35, -36, 0))
                .lineTo(new Vector2d(-36, -36))
                .build();

        TrajectorySequence jParkR = drive.trajectorySequenceBuilder(new Pose2d(-36, -36, 0))
                .lineTo(new Vector2d(-36, -60))
                .build();

        //step1: ехать вперед, начать поднимать лифт и ротейтить таур
        //step2: ехать в право на джанк; в это же время поднимать лифт до конца и отпускать конус
        //step3: ехать влево обратно шагу 2, начать спускать лифт и оборачивать таур на 180
        //step4: ехать назад, закончить поворот и продложить опускать лифт
        //step5: ехаьть в лево на стак, к концу опустить лифт и открыть кло
        //step6: ехать вправо, поднимая лифт и оборачивая таур на 180. В конце поворот базы на 45
        //step7: тупо стрейфрайт на джанкшн, продложить поднимать лифт и оборачивтаь таур на 180 . В конце отпустить кло
        //step8: тупо стрейфлефт, с опуском лифта и оборотом обратно на -180, в конце турн на -45 обратно
        //step: влево до стака, продолжить оборот клова на -180 и опуск лифта в стак конусов
        //step10: cycle

        //minimal rotation auton
        TrajectorySequence step1 = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setLift(1))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> setTower(-0.3))
                .UNSTABLE_addTemporalMarkerOffset(1.15, () -> setTower(0))
                .UNSTABLE_addTemporalMarkerOffset(1.22, () -> setLift(0))
                .lineTo(new Vector2d(3, -36))
                .build();

        TrajectorySequence step2 = drive.trajectorySequenceBuilder(new Pose2d(0, -36, 0))
                .lineTo(new Vector2d(2, -25))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-1))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> setClaw(false))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setLift(0.7))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> setLift(0))
                .waitSeconds(0.6)
                .build();

        TrajectorySequence step3 = drive.trajectorySequenceBuilder(new Pose2d(0, -27, 0))
                .lineTo(new Vector2d(0, -36))
                .build();

        TrajectorySequence step4 = drive.trajectorySequenceBuilder(new Pose2d(0, -36, 0))
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> setLift(-0.7))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setTower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> setTower(0))
                .UNSTABLE_addTemporalMarkerOffset(1.15, () -> setLift(0))
                .lineTo(new Vector2d(-12, -36))
                .build();

        TrajectorySequence step5 = drive.trajectorySequenceBuilder(new Pose2d(-12, -36, 0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setLift(-0.25))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setLift(0))
                .lineTo(new Vector2d(-12, -66))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-0.28))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setClaw(true))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setLift(0.5))
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> setLift(0))
                .waitSeconds(0.7)
                .build();

        TrajectorySequence step6 = drive.trajectorySequenceBuilder(new Pose2d(-12, -66))
                .lineTo(new Vector2d(-12, -36))
                .build();
//        TrajectorySequence balls3D = drive.trajectorySequenceBuilder(startPose)
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> lift.setPower(0.79 * kVol))
//                .UNSTABLE_addTemporalMarkerOffset(1.04, () -> lift.setPower(0.5 * kVol))
//                .UNSTABLE_addTemporalMarkerOffset(2.25, () -> lift.setPower(0))
//                .UNSTABLE_addDisplacementMarkerOffset(1.1, () -> setTower(0.6 * kVol))
//                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> setTower(0))
//                .splineToConstantHeading(new Vector2d(-36, 36), 0)
//                .splineToSplineHeading(new Pose2d(-12, 36, Math.toRadians(45)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(0, 24), Math.toRadians(-45))
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> lift.setPower(-1*kVol))
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> lift.setPower(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.3, ()-> claw.setPosition(0.9*kVol))
//                .waitSeconds(0.5)
//                .setTangent(Math.toRadians(135))
//
//                //junction to cones
//                .splineToSplineHeading(new Pose2d(-12, 40, Math.toRadians(270)), Math.toRadians(90))
//                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> setTower(-0.7))
//                .UNSTABLE_addTemporalMarkerOffset(0.9, () -> setTower(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> lift.setPower(-0.8))
//                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setPower(0))
//                .splineToConstantHeading(new Vector2d(-12, 65), Math.toRadians(90))
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> lift.setPower(-0.6))
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> lift.setPower(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> claw.setPosition(0.5))
//                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setPower(0.85))
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> lift.setPower(0))
//                .waitSeconds(0.6)
//                .setTangent(Math.toRadians(-90))
//                //cones to junction
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> lift.setPower(0.9))
//                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> lift.setPower(0.3))
//                .UNSTABLE_addTemporalMarkerOffset(1.6, () -> lift.setPower(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> setTower(-0.8))
//                .UNSTABLE_addTemporalMarkerOffset(1.05, () -> setTower(0))
//                .splineToConstantHeading(new Vector2d(-12, 41.5), Math.toRadians(-90))
//
//                .splineToSplineHeading(new Pose2d(3, 23, Math.toRadians(225)), Math.toRadians(-45))
//                .UNSTABLE_addTemporalMarkerOffset(0.05, () -> lift.setPower(-0.5))
//                .UNSTABLE_addTemporalMarkerOffset(0.35, () -> lift.setPower(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> claw.setPosition(0.9))
//                .waitSeconds(0.5)
//                .setTangent(Math.toRadians(180))
//
//                //junction to cones
//                .splineToSplineHeading(new Pose2d(-14, 40, Math.toRadians(270)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(90))
//                .waitSeconds(0.3)
//                .setTangent(Math.toRadians(-90))
//                //cones to junction
//                .splineToConstantHeading(new Vector2d(-14, 49), Math.toRadians(-90))
//                //.setTangent(Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(-1, 24, Math.toRadians(225)), Math.toRadians(-45))
//                .waitSeconds(0.3)
//                .setTangent(Math.toRadians(180))
//
//                //junction to cones
//                .splineToSplineHeading(new Pose2d(-14, 40, Math.toRadians(270)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-12, 65), Math.toRadians(90))
//                .waitSeconds(0.3)
//                .setTangent(Math.toRadians(-90))
//                //cones to junction
//                .splineToConstantHeading(new Vector2d(-14, 49), Math.toRadians(-90))
//                //.setTangent(Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(-1, 24, Math.toRadians(225)), Math.toRadians(-45))
//                .waitSeconds(0.3)
//                .setTangent(Math.toRadians(180))
//
//                //junction to cones
//                .splineToSplineHeading(new Pose2d(-14, 40, Math.toRadians(270)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-12, 65), Math.toRadians(90))
//                .waitSeconds(0.3)
//                .setTangent(Math.toRadians(-90))
//                //cones to junction
//                .splineToConstantHeading(new Vector2d(-14, 49), Math.toRadians(-90))
//                //.setTangent(Math.toRadians(270))
//                .splineToSplineHeading(new Pose2d(-1, 24, Math.toRadians(225)), Math.toRadians(-45))
//                .waitSeconds(0.3)
//                .setTangent(Math.toRadians(180))
//                .build();




        double liftHigh;
        double tower90;
        double liftCone;

        Vector2d junction = new Vector2d(0, 24);
        Vector2d cones = new Vector2d(-12, 70);

        //TrajectorySequence balls4D = drive.trajectorySequenceBuilder(startPose)

        telemetry.addLine("trajectories built 2");
        telemetry.update();
        boolean autontest = false;


        waitForStart();


        //drive.followTrajectorySequence(traj);
        if(!autontest) {
            if (foundTag == AprilTagsDetector.Tag.noTag || foundTag == AprilTagsDetector.Tag.left) {
                drive.followTrajectorySequence(leftParking);
            } else if (foundTag == AprilTagsDetector.Tag.mid) {
                drive.followTrajectorySequence(midParking);
            } else {
                drive.followTrajectorySequence(rightParking);
            }
        }
        else{
            drive.followTrajectorySequence(step1);
            drive.followTrajectorySequence(step2);
            drive.followTrajectorySequence(step3);
            drive.followTrajectorySequence(step4);
            //drive.followTrajectorySequence(step5); TEMP
            drive.followTrajectorySequence(junctionParkSetup);
            if (foundTag == AprilTagsDetector.Tag.noTag || foundTag == AprilTagsDetector.Tag.left) {
                drive.followTrajectorySequence(jParkL);
            } else if (foundTag == AprilTagsDetector.Tag.mid) {
                drive.followTrajectorySequence(jParkM);
            } else {
                drive.followTrajectorySequence(jParkR);
            }
        }
    }

    void setTower(double v) {
        double vScaled = clamp(v, 1);
        vScaled *= 0.25;
        tower.setPosition(0.5 + vScaled);
    }

    double clamp(double v, double bounds) {
        return clamp(v, -bounds, bounds);
    }

    double clamp(double v, double min, double max) {
        if (v > max) {
            return max;
        } else if (v < min) {
            return min;
        } else {
            return v;

        }
    }

    void setClaw(boolean open){
        if(open){
            claw.setPosition(0.5);
        }
        else{
            claw.setPosition(0.9);
        }
    }

    void setLift(double power){
        lift.setPower(power);
    }

}
