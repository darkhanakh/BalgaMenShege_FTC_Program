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


@Autonomous(name = "Left side autonomous 4")
public class leftAutonF extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(-65.1, 36, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        claw = hardwareMap.get(Servo.class, "claw");
        tower = hardwareMap.get(Servo.class, "tower");
        lift = hardwareMap.get(CRServo.class, "lift");

        while (opModeInInit()) {
            if (opModeInInit()) {
                setClaw(true);
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


        TrajectorySequence preload = drive.trajectorySequenceBuilder(startPose)
                //move from start closer to high junction, raise and rotate lift 90
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setLift(0.95))
                .UNSTABLE_addTemporalMarkerOffset(1.4, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setTower(1))
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> setTower(0))
                .lineToConstantHeading(new Vector2d(-3, 36))
                //move toward high junction, deposit cone
                .lineToConstantHeading(new Vector2d(-3, 26.5))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-0.7))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> setClaw(false))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> setLift(0.7))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> setLift(0))
                .waitSeconds(0.3)
                //move away from high junction to cone stack, lower and rotate lift -180
                .lineToConstantHeading(new Vector2d(-3 , 36))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setLift(-1))
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setTower(-0.85))
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> setTower(0))
                .lineToConstantHeading(new Vector2d(-12, 36))
                .lineToConstantHeading(new Vector2d(-12, 66.5))
                //grab cone
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-1))
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setClaw(true))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> setLift(1))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> setLift(0))

                .waitSeconds(0.6)
                .build();

        //mem xuyna, ne usay
//        TrajectorySequence altPreload = drive.trajectorySequenceBuilder(startPose)
//                .lineToConstantHeading(new Vector2d(-4, 36))
//                .splineToConstantHeading(new Vector2d(0, 27), Math.toRadians(-90))
//                .waitSeconds(0.5)
//                .setTangent(Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-4, 36), Math.toRadians(180))
//                .lineToConstantHeading(new Vector2d(-12, 36))
//                .lineToConstantHeading(new Vector2d(-12, 72))
//                .build();


        double loops = 0;
        boolean doReturn = false;

        TrajectorySequence stackToJunction = drive.trajectorySequenceBuilder(new Pose2d(-12, 66.5, 0))
                //drive from stack to junction, raise
                .UNSTABLE_addTemporalMarkerOffset(0, () -> setLift(0.8))
                //.UNSTABLE_addTemporalMarkerOffset(0.8 + (loops - 1) * 0.15, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> setTower(1))
                .UNSTABLE_addTemporalMarkerOffset(1.3, () -> setTower(0))
                .lineToConstantHeading(new Vector2d(-13, 44))
                .splineTo(new Vector2d(-1, 25), Math.toRadians(315))
                //deposit cone
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-0.7))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> setLift(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> setClaw(false))
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> setLift(0.7))
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> setLift(0))
//return to stack
                .setTangent(Math.toRadians(135))
//                            .waitSeconds(0.1)
//                            .UNSTABLE_addTemporalMarkerOffset(0.15, () -> setTower(-1))
//                            .UNSTABLE_addTemporalMarkerOffset(1.3, () -> setTower(0))//doesnt work???
                            .splineTo(new Vector2d(-12, 48), Math.toRadians(90))
//                            .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-1))
//                            .UNSTABLE_addTemporalMarkerOffset(0.625, () -> setLift(0))
//                            .lineToConstantHeading(new Vector2d(-12, 67.5))
//                //pick up cone,
//                            .waitSeconds(0.2)
//                            .UNSTABLE_addTemporalMarkerOffset(0.05, () -> setLift(-0.6))
//                            .UNSTABLE_addTemporalMarkerOffset(0.3, () -> setLift(0))
//                            .UNSTABLE_addTemporalMarkerOffset(0.3, () -> setClaw(true))
//                            .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setLift(0.75))
//                            .UNSTABLE_addTemporalMarkerOffset(0.7, () -> setLift(0))
//                            .waitSeconds(0.8)
//
//
//
                .build();

        Pose2d endPose = new Pose2d(new Vector2d(-1, 25), Math.toRadians(315));

        TrajectorySequence leftParking = drive.trajectorySequenceBuilder(endPose)
//                .forward(53)
                .lineTo(new Vector2d(-12, 60))
                .build();

        TrajectorySequence midParking = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-12, 34))
                .build();

        TrajectorySequence rightParking = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-12, 12))
                .build();
//
//
//
//
//        TrajectorySequence junctionToStack = drive.trajectorySequenceBuilder(new Pose2d(-1, 25, Math.toRadians(315)))
//                .setTangent(Math.toRadians(135))
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-0.8))
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> setLift(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setTower(-1))
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> setTower(0))
//                .splineTo(new Vector2d(-12, 48), Math.toRadians(90))
//                .lineToConstantHeading(new Vector2d(-12, 66.5))
//
//                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> setLift(-0.8))
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> setLift(0))
//                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> setClaw(true))
//                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> setLift(1))
//                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> setLift(0))
//
//                .waitSeconds(0.6)
//                .build();

        telemetry.addLine("trajectories built 2");
        telemetry.update();
        boolean autontest = false;


        waitForStart();
        drive.followTrajectorySequence(preload);
        drive.followTrajectorySequence(stackToJunction);
        if(!autontest) {
            if (foundTag == AprilTagsDetector.Tag.noTag || foundTag == AprilTagsDetector.Tag.left) {
                drive.followTrajectorySequence(leftParking);
            } else if (foundTag == AprilTagsDetector.Tag.mid) {
                drive.followTrajectorySequence(midParking);
            } else {
                drive.followTrajectorySequence(rightParking);
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
            claw.setPosition(0.1);
        }
        else{
            claw.setPosition(0.2);
        }
    }

    void setLift(double power){
        lift.setPower(power);
    }

}
