package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.hardware.KS109I2cDistance;
import org.firstinspires.ftc.teamcode.common.powerplay.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="Chris_1D_Left", group="auto")
public class Chris_1D_Left extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    // Mobor encoder parameter
    double ticksPerInch = 6.56;
    double ticksPerDegree = 5.0;

    // Tag id
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    int tagID = MIDDLE;

    IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    private DcMotor motorfl;
    private DcMotor motorbl;
    private DcMotor motorfr;
    private DcMotor motorbr;

    private DcMotor armR;
    private DcMotor armL;

    private Servo grab;
    private Servo tilt;

    private ColorSensor colorSens;
    private DistanceSensor distanceSens, distanceSens2;
    private KS109I2cDistance ks109;  // Hardware Device Object

    private int flPos;
    private int blPos;
    private int frPos;
    private int brPos;

    private int armPos;

    private List<DcMotor> motors;
    private List<DcMotor> arms;

    // Initial robot orientation
    YawPitchRollAngles orientation0;
    AngularVelocity angularVelocity0;
    double yaw0;


    @Override
    public void runOpMode() {
        long maxAllowedTimeInMills = 2000;
        // Initialize IMU in the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");

        armR = hardwareMap.get(DcMotor.class, "armR");
        armL = hardwareMap.get(DcMotor.class, "armL");

        grab = hardwareMap.get(Servo.class, "grabServo");
        tilt = hardwareMap.get(Servo.class, "tiltServo");
        ks109 = hardwareMap.get(KS109I2cDistance.class, "ks109");
        distanceSens = hardwareMap.get(DistanceSensor.class, "distanceSens");
        distanceSens2 = hardwareMap.get(DistanceSensor.class, "distanceSens2");


        motors = Arrays.asList(motorfl, motorbl, motorbr, motorfr); //array containing all motors
        arms = Arrays.asList(armL, armR);//array of arm motors

        motorfr.setDirection(DcMotor.Direction.REVERSE); //reverse motor direction
        armL.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotor arm: arms){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Retrieve the very initial Rotational Angles and Velocities
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);

        flPos = 0;
        blPos = 0;
        frPos = 0;
        brPos = 0;

        armPos = 0;

        // April Tag code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);}
            @Override public void onError(int errorCode) {}});
        telemetry.setMsTransmissionInterval(50);

        // Detect AprilTag after initialization and before Start is pressed.
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagID = tag.id;
                        telemetry.addLine(String.format("\nDetected tag ID=%d, Distance=%.2f inch, yaw0=%.2f, yaw=%.2f degree",
                                tagID, ks109.getDistance(), yaw0, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
                        break;
                    }
                }
            }
            telemetry.update();
            sleep(20);
        }
        // Either Started or Stopped.
        // Show detected TagID
        telemetry.addLine(String.format("\n\nFinal detected tag ID=%d", tagID));
        telemetry.update();
        if (isStopRequested()) {
            // Stop
            return;
        }

        // Stop AprilTag recognition
        camera.closeCameraDevice();
        aprilTagDetectionPipeline.finalize();
        camera = null;
        aprilTagDetectionPipeline = null;

        // grab the preloaded cone
        grab.setPosition(0.7);
        sleep(500);
        driveArm(5,0.7);
        sleep(500);


        // push signal cone away - move forward first and then move backward to be away from the signal cone.
        // Field test data:
        //  1) 400 ticks == 61 inch  ====> 1 inch == 6.56 ticks
        //  2) Power 0.6 is the most stable power level
        double power = 0.3;
        double factor = 1;
        driveMotors(420,420,420,420, power, true, yaw0);
        sleep(500);

        // move backward to 52 (+- 0.5) inches
        maxAllowedTimeInMills = 3000;
        driveMotorsToDistance(51, power, true, yaw0, maxAllowedTimeInMills);
        //driveMotors(420,420,420,420,power);

        // turn right 45 degrees
        int turnTicks = 80;
        double turnPower = 0.3;
        driveMotors(turnTicks, turnTicks, -turnTicks, -turnTicks, turnPower, false, 0);
        sleep(500);
//        sleep(5000);

        // place cone and low arm to XX from final high position YY
        // The end position must be calculated based on previous position.
        placeCone(70,-60, 1000, 0.6, 0.5);

        // This is the major difference between 1A and 1B.
        // Robot will turn right for about 45 degrees with its back facing the wall in order to:
        //  1) Reduce the turn error due to the bigger rotation.
        //  2) Utilize ks109 distance sensor for more precise parking to the destination zone.
        //
        // Turn right for exactly 90 degree using IMU sensor.
        turnToTargetYaw(-90 + yaw0, turnPower, maxAllowedTimeInMills);
        sleep(500);

        //
        // Park to the identified zone accurately by using ks109 sensor...
        //
        double targetDistanceInch;
        int moveTicks = 0;
        if (tagID == LEFT){
//            moveTicks = -140;
//            driveMotors(moveTicks,moveTicks,moveTicks,moveTicks,0.2);
            targetDistanceInch = 5.0;
            maxAllowedTimeInMills = 2000;
        }
        else if (tagID == MIDDLE){
            targetDistanceInch = 27.0;
            maxAllowedTimeInMills = 5000;
        }
        else{// tagID == RIGHT
//            moveTicks = 150;
//            driveMotors(moveTicks,moveTicks,moveTicks,moveTicks,0.2);
            targetDistanceInch = 48.0;
            maxAllowedTimeInMills = 5000;
        }
        driveMotorsToDistance(targetDistanceInch, 0.2, true, -90 + yaw0, maxAllowedTimeInMills);
        sleep(1000);
        requestOpModeStop();
    }


    private void driveWithoutEncoders(double flPower, double blPower, double frPower, double brPower){
        for (DcMotor motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motorfl.setPower(flPower);
        motorbl.setPower(blPower);
        motorfr.setPower(frPower);
        motorbr.setPower(brPower);

    }
    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget, double speed,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfl.setTargetPosition(flTarget);
        motorbl.setTargetPosition(blTarget);
        motorfr.setTargetPosition(frTarget);
        motorbr.setTargetPosition(brTarget);

        motorfl.setPower(speed);
        motorbl.setPower(speed);
        motorfr.setPower(speed);
        motorbr.setPower(speed);

        motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (motorfl.isBusy() && motorbl.isBusy() && motorfr.isBusy() && motorbr.isBusy())){
            if (bKeepYaw) {
                currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = speed * (1 - direction * powerDeltaPct);
                    powerR = speed * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = speed * (1 + direction * powerDeltaPct);
                    powerR = speed * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                motorfl.setPower(powerL);
                motorbl.setPower(powerL);
                motorfr.setPower(powerR);
                motorbr.setPower(powerR);
            }
            idle();
        }

        motorfl.setPower(0);
        motorbl.setPower(0);
        motorfr.setPower(0);
        motorbr.setPower(0);
    }

    private void driveMotorsToDistance(double targetDistanceInch, double power, boolean bKeepYaw, double targetYaw, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentDistance = ks109.getDistance();
        int ticks, tickDirection;
        double diff = Math.abs(currentDistance - targetDistanceInch);
        telemetry.addLine(String.format("\nDistance=%.2f inch", currentDistance));
        telemetry.update();
        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diff > 0.7
                && opModeIsActive()
                && ((timeCurrent - timeBegin) < maxAllowedTimeInMills)){
            ticks = (int) (diff * ticksPerInch);
            tickDirection = (currentDistance < targetDistanceInch) ? 1 : -1;
            if (ticks > 0) {
                driveMotors((int)(tickDirection * ticks), (int)(tickDirection * ticks),
                        (int)(tickDirection * ticks), (int)(tickDirection * ticks),
                        power, bKeepYaw, targetYaw);
                currentDistance = ks109.getDistance();
            }
            telemetry.addLine(String.format("\nDistance=%.2f inch", currentDistance));
            telemetry.update();
            diff = Math.abs(currentDistance - targetDistanceInch);
            timeCurrent = System.currentTimeMillis();
        }
    }

    private void driveOneMotor(DcMotor motor, int position, double power){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(position);
        motor.setPower(power);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && motor.isBusy()) {
            idle();
        }
        motor.setPower(0);
    }

    private void driveArm(int armTarget, double power){
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setTargetPosition(armTarget);
        armR.setTargetPosition(armTarget);
        armL.setPower(power);
        armR.setPower(power);
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(opModeIsActive() && (armR.isBusy() && armL.isBusy())){
            idle();
        }
//        for (DcMotor arm: arms){
//            arm.setPower(0);
//        }
    }

    public void placeCone(int target, int end, int intervalMs, double wheelPower, double armPower){
        long timeBegin, timeCurrent, maxAllowedTimeInMills=2000;
        double yawBegin;    // yaw degrees detected a junction
        double yawEnd;      // yaw degrees lost a junction;
        double yawMid;      // yaw degrees face to the middle of the junction.
        double yawJunction; // yaw degrees of a junction

        // Raise arm
        driveArm(target, armPower);
        sleep(intervalMs);
        // set proper tilt angle
        tilt.setPosition(0.7);
        sleep(intervalMs);

        // Turn right slowly
        // Record yawBegin when the Rev2M distance sensor detected a distance less than 20 inches
        // Record yawEnd when the distance is bigger than 20 inches.
        // So that yawMid can be used to precisely orient the robot.
        yawBegin = yawEnd = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        timeBegin = timeCurrent = System.currentTimeMillis();
        driveWithoutEncoders(0.3,0.3,-0.3,-0.3);
        while (opModeIsActive()
                && ((timeCurrent - timeBegin) < maxAllowedTimeInMills)){
            if (distanceSens.getDistance(DistanceUnit.INCH)<24){
                // Record yawBegin. Do not stop the robot turning.
                yawBegin = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                driveWithoutEncoders(0,0,0,0);
                sleep(200);
                 break;
            }
            timeCurrent = System.currentTimeMillis();
            sleep(20);
        }
//
//        while (opModeIsActive()){
//            if (distanceSens.getDistance(DistanceUnit.INCH)>20){
//                // Record yawBegin. STOP the robot turning.
//                yawEnd = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//                driveWithoutEncoders(0,0,0,0);
//                break;
//            }
//        }
        // Calculate yawMid
        //  r = radius = 16 inch (sensor-to-junction) + 2 inch (sensor-to-robot-center)
        //  p = perimeter = 2 * PI * r
        //  d = diameter of junction = 1 inch
        yawEnd = yawBegin + (1 / (2 * 3.1415926 * 18)) * 360;
        yawMid = yawBegin + (yawEnd - yawBegin) * 0.75;
//        yawBegin = yawMid = yawEnd = -46 + yaw0;

        telemetry.addLine(String.format("yaw0=%.2f\nyawBegin=%.2f\nyawEnd=%.2f\nyawMid=%.2f\nyawCurrent=%.2f",
                yaw0, yawBegin, yawEnd, yawMid, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        telemetry.update();
//        turnToTargetYaw(yawMid, wheelPower, maxAllowedTimeInMills);
//        telemetry.addLine(String.format("yaw0=%.2f\nyawBegin=%.2f\nyawEnd=%.2f\nyawMid=%.2f\nyawCurrent=%.2f",
//                yaw0, yawBegin, yawEnd, yawMid, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
//        telemetry.update();

        // move forward till X inches
        // use two distance sensors to improve reliability
        double powerRatio = 1.0;
        driveWithoutEncoders(0.3,0.3,0.3,0.3);
        double targetDistance = 12.5, currentDistance;
        timeBegin = timeCurrent = System.currentTimeMillis();
        while (opModeIsActive()
                && ((timeCurrent - timeBegin) < maxAllowedTimeInMills)){
            currentDistance = Math.min(
                    distanceSens.getDistance(DistanceUnit.INCH),
                    distanceSens2.getDistance(DistanceUnit.INCH));
            if (currentDistance < targetDistance){
                //sleep(200);
                driveWithoutEncoders(0,0,0,0);
                break;
            }
            if (Math.abs(currentDistance - targetDistance) > 2.0)
                powerRatio = 1.0;
            else
                powerRatio = Math.abs(currentDistance - targetDistance) / 2.0;
            driveWithoutEncoders(0.3 * powerRatio,0.3 * powerRatio,0.3 * powerRatio,0.3 * powerRatio);
            timeCurrent = System.currentTimeMillis();
        }

        tilt.setPosition(1.0);
        sleep(intervalMs);

        // loose the grabber
        grab.setPosition(-0.5);
        sleep(intervalMs);

        // driver back
        int backwardTicks = -40;
        driveMotors(backwardTicks, backwardTicks, backwardTicks, backwardTicks, wheelPower, false, 0);
        sleep(intervalMs);

        // close the grabber
        grab.setPosition(0.5);
        sleep(intervalMs);

        // Lower arm to the target position
        driveArm(end, armPower);
        sleep(intervalMs);
    }

    private void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 50)
                ticks = 50;
            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }

}

