package org.firstinspires.ftc.teamcode.custom.driving;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp(name = "MainDrive")
public class MainDrive extends LinearOpMode {
    private Motor frontLeftMotor = null;
    private Motor backLeftMotor = null;
    private Motor frontRightMotor = null;
    private Motor backRightMotor = null;
    private Servo claw = null;
    private CRServo lift = null;
    private Servo tower = null;
    private final ElapsedTime liftBind = new ElapsedTime();
    private ColorSensor front;
    ColorSensor right;
    ColorSensor back;
    ColorSensor left;
    private boolean lbPrevState = false;
    private BNO055IMU imu = null;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    public static double kP = 0.018, kI = 0, kD = 0;
    public static double kPTower = 0.00037;
    private boolean cascadeGoingUpH = false;
    private boolean cascadeGoingUpM = false;
    private boolean cascadeGoingUpL = false;
    private boolean isClawOpen = false;

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
        frontLeftMotor = new Motor(hardwareMap, "leftFront");
        backLeftMotor = new Motor(hardwareMap, "leftRear");
        frontRightMotor = new Motor(hardwareMap, "rightFront");
        backRightMotor = new Motor(hardwareMap, "rightRear");

        claw = hardwareMap.get(Servo.class, "claw");
        tower = hardwareMap.get(Servo.class, "tower");
        lift = hardwareMap.get(CRServo.class, "lift");
        front = hardwareMap.get(ColorSensor.class, "front");
        right = hardwareMap.get(ColorSensor.class, "right");
        left = hardwareMap.get(ColorSensor.class, "left");
        back = hardwareMap.get(ColorSensor.class, "back");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        frontLeftMotor.setRunMode(Motor.RunMode.RawPower);
        frontRightMotor.setRunMode(Motor.RunMode.RawPower);
        backLeftMotor.setRunMode(Motor.RunMode.RawPower);
        backRightMotor.setRunMode(Motor.RunMode.RawPower);

        PIDFController ang = new PIDFController(kP, kI, kD, 0);
        HDrive drive = new HDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        double savedHeading = 0.0;
        double targetHeading = -90;
        double turnRate = -180;
        double pHeading = 0.15;
        double errHeading = 0.0;
        double deltaHeading = 0.0;
        double depression = 0;

        int trgTower = 2; //1 right, 2 fwd, 3 left, 4 back
        int trgTowerPrev = 2;
        int dir = 1;
        int dir2 = 1;
        int targetTowerVal = 2850;
        int error = 0;
        int errorPrev = 0;
        int errorDelta = 0;
        int readingTower = 0;

        double lastHeading = 0.0;
        ElapsedTime timer = new ElapsedTime();
        double prevTime = 0;

        boolean clawPrev = false;

        ElapsedTime tick = new ElapsedTime();
        GamepadEx driverOp = new GamepadEx(gamepad1);
        ang.setP(kP);
        ang.setI(kI);
        ang.setD(kD);

        if (isStopRequested())
            return;
        waitForStart();
        double deltaTime = 0;
        double timerS = 0;
        double kV = voltage();
        boolean lTLast = false;
        boolean clawOpen = false;
        double clawPos = 0.5;
        while (opModeIsActive()) {
            double xStick = driverOp.getLeftX();
            double yStick = driverOp.getLeftY();
            double velo = 1;
            boolean lT = gamepad2.left_trigger > 0.2;
            if (Math.abs(xStick) > 0.75) {
                xStick = Math.signum(xStick);
            }
            if (Math.abs(yStick) > 0.75) {
                yStick = Math.signum(yStick);
            }
            velo = 1 - driverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.7;

            double liftVel = 0;
            if (!lTLast && lT) {
                clawOpen = !clawOpen;
            }
            if(gamepad2.left_bumper && !clawPrev){
                clawOpen = !clawOpen;
            }
            clawPrev = gamepad2.left_bumper;
            if (clawOpen) {
                clawPos = 0.9;
            } else {
                clawPos = 0.5;
            }
            claw.setPosition(clawPos);
            lTLast = lT;

            //set the target position for the tower based on driver input
            if (gamepad1.y) {
                trgTowerPrev = trgTower;
                trgTower = 4;
            }
            if (gamepad1.x) {
                trgTowerPrev = trgTower;
                trgTower = 1;
            }
            if (gamepad1.b) {
                trgTowerPrev = trgTower;
                trgTower = 3;
            }
            if (gamepad1.a) {
                trgTowerPrev = trgTower;
                trgTower = 2;
            }
            if (gamepad2.right_bumper) {
                liftVel = 0.85;
            } else if (gamepad2.right_trigger > 0.2) {
                liftVel = -0.650
                ;
            } else {
                liftVel = 0;
            }
            if (gamepad2.right_stick_x > 0) {
                tower.setPosition(0.6);
            } else if (gamepad2.right_stick_x < 0) {
                tower.setPosition(0.4);
            }
            if (gamepad2.b)
                if (!lbPrevState) {
                    cascadeGoingUpH = true;
                    liftBind.reset();
                } else lbPrevState = false;
            if (cascadeGoingUpH)
                if (liftBind.seconds() <= 1.2)
                    lift.setPower(1 * kV);
                else {
                    lift.setPower(0);
                    cascadeGoingUpH = false;
                }
            if (gamepad2.y)
                if (!lbPrevState) {
                    cascadeGoingUpM = true;
                    liftBind.reset();
                } else lbPrevState = false;
            if (cascadeGoingUpM)
                if (liftBind.seconds() <= 1.1)
                    lift.setPower(1 * kV);
                else {
                    lift.setPower(0);
                    cascadeGoingUpM = false;
                }
            if (gamepad2.x)
                if (!lbPrevState) {
                    cascadeGoingUpL = true;
                    liftBind.reset();
                } else lbPrevState = false;
            if (cascadeGoingUpL)
                if (liftBind.seconds() <= 0.65)
                    lift.setPower(1 * kV);
                else {
                    lift.setPower(0);
                    cascadeGoingUpL = false;
                }


            switch (trgTower) {
                case 2:
                    readingTower = front.alpha();
                    break;
                case 1:
                    readingTower = right.alpha();
                    break;
                case 4:
                    readingTower = back.alpha();
                    break;
                case 3:
                    readingTower = left.alpha();
                    break;
            }
            //calculate absolute and relative heading
            double absoluteHeading = -90 + getAngle();

            double relativeHeading = absoluteHeading - savedHeading;
            if (gamepad1.right_bumper) {
                resetAngle();
                targetHeading = -90;
            }
            //set the target heading
            //if (driverOp.getRightY() * driverOp.getRightY() + driverOp.getRightX() * driverOp.getRightX() > 0.4) {
            //targetHeading = Math.toDegrees(Math.atan2(-driverOp.getRightX(), -driverOp.getRightY()));
            //}1
            if (Math.abs(driverOp.getRightX()) > 0.15) {
                targetHeading += driverOp.getRightX() * turnRate * deltaTime / 1000;
            }
            if (targetHeading < -180) {
                targetHeading += 360;
            } else if (targetHeading > 180) {
                targetHeading -= 360;
            }
            if ((relativeHeading > 90 && targetHeading < -90) || (relativeHeading < -90 && targetHeading > 90)) {
                dir2 = -1;
            } else {
                dir2 = 1;
            }
            if (Math.abs(ang.getPositionError()) > 300) {
                depression = 0.65;
            } else {
                depression = 1;
            }
            //lift turn direction
            if (trgTower > trgTowerPrev) {
                dir = 1;
                //targetTowerVal = -Math.abs(targetTowerVal);
            }
            if (trgTower < trgTowerPrev) {
                dir = -1;
                //targetTowerVal = Math.abs(targetTowerVal);
            }

            if (errorDelta < -2500) {
                dir = -1;
            }

            //calculate error and pid
            errHeading = targetHeading - relativeHeading;
            double turn = ang.calculate(relativeHeading, targetHeading) * dir2 * depression;
            error = targetTowerVal - readingTower;
            double vel = kPTower * error * dir;

            //set velocities
            if (!gamepad1.dpad_up) {
                setTower(vel);
            } else {
                setTower(0);
            }
            drive.driveFieldCentric(-yStick * velo, xStick * velo, turn, relativeHeading);
            lift.setPower(liftVel);
            //calculate deltas
            deltaHeading = relativeHeading - lastHeading;
            lastHeading = relativeHeading;
            errorDelta = error - errorPrev;
            errorPrev = error;
            //tele
            telemetry.addLine("Heading");
            telemetry.addData("absolute", absoluteHeading);
            telemetry.addData("relative", relativeHeading);
            telemetry.addData("target", targetHeading);
            telemetry.addData("error", errHeading);
            telemetry.addData("kP * error * kms", kP * errHeading * depression);
            telemetry.addData("delta", deltaHeading);

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Tower");
            telemetry.addData("target", targetTowerVal);
            telemetry.addData("error", error);
            telemetry.addData("kP * error", kPTower * error);
            telemetry.addData("error delta", errorDelta);

            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.addLine("Claw");
            telemetry.addData("pos", claw.getPosition());

            telemetry.update();
            deltaTime = timer.milliseconds() - timerS;
            timerS = timer.milliseconds();
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

    void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle_ = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle_ < -180)
            deltaAngle_ += 360;
        else if (deltaAngle_ > 180)
            deltaAngle_ -= 360;

        globalAngle += deltaAngle_;

        lastAngles = angles;

        return globalAngle;
    }

}