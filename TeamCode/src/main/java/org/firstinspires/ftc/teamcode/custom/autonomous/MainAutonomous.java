package org.firstinspires.ftc.teamcode.custom.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.custom.driving.MainDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "Main autonomous")
public class MainAutonomous extends LinearOpMode {
    private Servo claw = null;
    private CRServo lift = null;
    private Servo tower = null;

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
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-65.1, 36, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        claw = hardwareMap.get(Servo.class, "claw");
        tower = hardwareMap.get(Servo.class, "tower");
        lift = hardwareMap.get(CRServo.class, "lift");

        if(isStopRequested()) return;

        double kVol = 12.8/voltage();

        telemetry.addLine("trajectories building...");
        telemetry.addData("kvol", kVol);
        telemetry.update();
//        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
//                .splineToSplineHeading(new Pose2d(-45, 36, 0), -0)
//                .splineToSplineHeading(new Pose2d(-24.1, 36, Math.toRadians(135)), 0)
//                .splineToSplineHeading(new Pose2d(-3.5,25.5,Math.toRadians(135)),Math.toRadians(-45))
//                .waitSeconds(1)
//                .splineToSplineHeading(new Pose2d(-15,46, Math.toRadians(0)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-15, 64), Math.toRadians(90))
//                .build();
//
//        TrajectorySequence seqs = drive.trajectorySequenceBuilder(startPose)
//                .splineToSplineHeading(new Pose2d(-36, 36, 0), 0)
//                .splineToSplineHeading(new Pose2d(-24, 36, 45), 0)
//                .splineToConstantHeading(new Vector2d(-3.0,21.5),Math.toRadians(-45))
//                .waitSeconds(0.8)
//                .splineToSplineHeading(new Pose2d(-15, 46, 135), Math.toRadians(90))
//
//                .build();
//
//        TrajectorySequence coqs = drive.trajectorySequenceBuilder(startPose)
//
//                .splineToSplineHeading(new Pose2d(-36, 36, 0), 0)
//                .splineToSplineHeading(new Pose2d(-24, 36, Math.toRadians(45)), 0)
//                .splineToConstantHeading(new Vector2d(-6.0,26.5), Math.toRadians(-45))
//                //start to junction
//                .splineToSplineHeading(new Pose2d(-10, 48, 0), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-10, 67), Math.toRadians(90))
//                //.waitSeconds(0.8)
//                //junction to stack
//                .splineToConstantHeading(new Vector2d(-10, 56), Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(-6, 26.5,
//                        Math.toRadians(-45)), Math.toRadians(-45))
//                //.waitSeconds(0.8)
//                //stack to junction
//                .splineToSplineHeading(new Pose2d(-10, 48, 0), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-10, 67), Math.toRadians(90))
//                //.waitSeconds(0.8)
//                //junction to stack 2
//                .splineToConstantHeading(new Vector2d(-10, 56), Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(-6, 26.5, Math.toRadians(-45)), Math.toRadians(-45))
//                .build();
//
//        TrajectorySequence adil1 = drive.trajectorySequenceBuilder(startPose)
//                .splineToConstantHeading(new Vector2d(-12, 36), Math.toRadians(0))
//                        .build();
//        TrajectorySequence adil2 = drive.trajectorySequenceBuilder((new Pose2d(-12, 36, 0)))
//                .splineToConstantHeading(new Vector2d(0, 28), Math.toRadians(-90))
//                .build();
//        TrajectorySequence adil3 = drive.trajectorySequenceBuilder(new Pose2d()).setReversed(true)
//                .splineToConstantHeading(new Vector2d(0,28), Math.toRadians(-90))
//                .build();
//
//        TrajectorySequence balls = drive.trajectorySequenceBuilder(startPose)
//                .splineToConstantHeading(new Vector2d(-36, 36), 0)
//                .addTemporalMarker(0.1, () -> lift.setPower(1))
//                .addTemporalMarker(1.3, () -> lift.setPower(0))
//                .splineToSplineHeading(new Pose2d(-12, 36, Math.toRadians(45)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(-3, 26.5), Math.toRadians(-45))
//                .waitSeconds(0.5)
//
//                //junction to cones
//                .strafeLeft(6)
//                .splineToSplineHeading(new Pose2d(-12, 48, Math.toRadians(270)), Math.toRadians(90))
//                .back(16)
//                .waitSeconds(0.5)
//                //cones to junction
//                .forward(16)
//                .splineToSplineHeading(new Pose2d(-11, 34.5, Math.toRadians(225)), Math.toRadians(-90))
//                .strafeLeft(14)
//                        .build();



        TrajectorySequence balls3D = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> claw.setPosition(0.5))
                .waitSeconds(1.3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> lift.setPower(0.9))
                .UNSTABLE_addTemporalMarkerOffset(1.12, () -> lift.setPower(0.45))
                .UNSTABLE_addTemporalMarkerOffset(2.25, () -> lift.setPower(0))
                .UNSTABLE_addDisplacementMarkerOffset(1.1, () -> setTower(0.5))
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> setTower(0))
                .splineToConstantHeading(new Vector2d(-36, 36), 0)
                .splineToSplineHeading(new Pose2d(-12, 36, Math.toRadians(45)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(2, 15), Math.toRadians(-45))
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> claw.setPosition(0.9))
                .waitSeconds(0.2)
                .setTangent(Math.toRadians(135))

                //junction to cones
                .splineToSplineHeading(new Pose2d(-15, 40, Math.toRadians(270)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> setTower(-0.55))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> setTower(0))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> lift.setPower(-0.9))
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> lift.setPower(0))
                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(90))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(-90))
                //cones to junction
                .splineToConstantHeading(new Vector2d(-12, 41.5), Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-5, 23, Math.toRadians(225)), Math.toRadians(-45))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(180))

                //junction to cones
                .splineToSplineHeading(new Pose2d(-14, 40, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-12, 60), Math.toRadians(90))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(-90))
                //cones to junction
                .splineToConstantHeading(new Vector2d(-14, 49), Math.toRadians(-90))
                //.setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-1, 24, Math.toRadians(225)), Math.toRadians(-45))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(180))

                //junction to cones
                .splineToSplineHeading(new Pose2d(-14, 40, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-12, 65), Math.toRadians(90))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(-90))
                //cones to junction
                .splineToConstantHeading(new Vector2d(-14, 49), Math.toRadians(-90))
                //.setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-1, 24, Math.toRadians(225)), Math.toRadians(-45))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(180))

                //junction to cones
                .splineToSplineHeading(new Pose2d(-14, 40, Math.toRadians(270)), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-12, 65), Math.toRadians(90))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(-90))
                //cones to junction
                .splineToConstantHeading(new Vector2d(-14, 49), Math.toRadians(-90))
                //.setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-1, 24, Math.toRadians(225)), Math.toRadians(-45))
                .waitSeconds(0.3)
                .setTangent(Math.toRadians(180))

                .build();

        telemetry.addLine("trajectories built 2");
        telemetry.update();
        waitForStart();

        //drive.followTrajectorySequence(traj);
        drive.followTrajectorySequence(balls3D);
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

}
