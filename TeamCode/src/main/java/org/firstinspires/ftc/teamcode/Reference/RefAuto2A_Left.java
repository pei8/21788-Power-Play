package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.driver.KS109I2cDistance;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// Target to score 2 elements
@Autonomous(name="RefAuto2A_Left", group="auto")
public class RefAuto2A_Left extends LinearOpMode {

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

    // Tag id
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;
    int tagID = MIDDLE;

    BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    private DcMotor motorfl;
    private DcMotor motorbl;
    private DcMotor motorfr;
    private DcMotor motorbr;

    private DcMotor armR;
    private DcMotor armL;

    private CRServo grab;
    private CRServo tilt;

    private ColorSensor colorSens;
    private DistanceSensor distanceSens;
    private KS109I2cDistance ks109;  // Hardware Device Object

    private int flPos;
    private int blPos;
    private int frPos;
    private int brPos;

    private int armPos;

    private List<DcMotor> motors;
    private List<DcMotor> arms;


    @Override
    public void runOpMode() {
        // Initialize IMU in the control hub
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");

        armR = hardwareMap.get(DcMotor.class, "armR");
        armL = hardwareMap.get(DcMotor.class, "armL");

        grab = hardwareMap.get(CRServo.class, "grab");
        tilt = hardwareMap.get(CRServo.class, "tilt");
        ks109 = hardwareMap.get(KS109I2cDistance.class, "ks109");

        motors = Arrays.asList(motorfl, motorbl, motorbr, motorfr); //array containing all motors
        arms = Arrays.asList(armL, armR);//array of arm motors

        motorfr.setDirection(DcMotor.Direction.REVERSE); //reverse motor direction
        motorbl.setDirection(DcMotor.Direction.REVERSE);
        armL.setDirection(DcMotor.Direction.REVERSE);
        grab.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        for (DcMotor arm: arms){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

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
                        telemetry.addLine(String.format("\nDetected tag ID=%d, Distance=%.2f inch",
                                tagID, ks109.getDistance()));
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
        grab.setPower(-0.5);
        sleep(500);
        driveArm(5,0.7);
        sleep(500);


        // push signal cone away - move forward first and then move backward to be away from the signal cone.
        // Field test data:
        //  1) 400 ticks == 61 inch  ====> 1 inch == 6.56 ticks
        //  2) Power 0.6 is the most stable power level
        driveMotors(400,400,400,400,0.6);
        sleep(500);

        // move backward XX 52 (+- 0.5) inches using KS109 sensor to measure the distance
        driveMotorsToDistance(51.5, 0.6);
//        sleep(5000);
//        requestOpModeStop();

        // turn right 45 degrees
        int turnTicks = 115;
        driveMotors(turnTicks, turnTicks, -turnTicks, -turnTicks, 0.6);
        sleep(500);

//        sleep(5000);
//        requestOpModeStop();

        // place cone and low arm from the highest position XX to 17
        // 17 is the arm position to the 5th cone in the stack based on the testing.
        // The end position must be calculated based on previous position.
        int highArmTicks = 60;      // from ground
        int fifthArmTicks = 18;     // from ground
        placeCone(highArmTicks, -(highArmTicks-fifthArmTicks), 500, 0.6, 0.5);
//
        // turn left 135 degrees. Now the robot is facing the wall and should be watching the stack of cones.
        turnTicks = 272;
        driveMotors(-turnTicks, -turnTicks, turnTicks, turnTicks, 0.6);
        sleep(500);

//        requestOpModeStop();
        int moveTicks = (int) (18 * ticksPerInch);
        driveMotors(moveTicks, moveTicks, moveTicks, moveTicks, 0.6);
        requestOpModeStop();




        if(tagID == LEFT) {
            // Move forward XX inches
            moveTicks = (int) (18 * ticksPerInch);
        }
        else if(tagID == RIGHT) {
            // Move backward YY inches
            moveTicks = -(int)(23 * ticksPerInch);
        }
        else { // Default, middle,
            // Move backward ZZ inches
            moveTicks = -(int)(2 * ticksPerInch);
        }
        driveMotors(moveTicks, moveTicks, moveTicks, moveTicks, 0.6);
        // tilt the grabber inward a little bit
        tilt.setPower(0.5);
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
    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget, double speed){
        flPos = flTarget;
        blPos = blTarget;
        frPos = frTarget;
        brPos = brTarget;


        for (DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        motorfl.setTargetPosition(flPos);
        motorbl.setTargetPosition(blPos);
        motorfr.setTargetPosition(frPos);
        motorbr.setTargetPosition(brPos);

        // set the desired power for all motors
        for (DcMotor motor: motors){
            motor.setPower(speed);
        }
        for (DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        while(opModeIsActive() &&
                (motorfl.isBusy() && motorbl.isBusy() && motorfr.isBusy() && motorbr.isBusy())){
            idle();
        }
        for (DcMotor motor: motors){
            motor.setPower(0);
        }
    }

    private void driveMotorsToDistance(double targetDistanceInch, double power){
        double currentDistance = ks109.getDistance();
        int ticks = 0;
        int tickDirection = 1;

        double diff = Math.abs(currentDistance - targetDistanceInch);
        telemetry.addLine(String.format("\nDistance=%.2f inch", currentDistance));
        telemetry.update();
        while (diff > 1.0 && opModeIsActive()) {
            ticks = (int) (diff * ticksPerInch);
            tickDirection = (currentDistance < targetDistanceInch) ? 1 : -1;
            if (ticks > 0) {
                driveMotors(tickDirection * ticks, tickDirection * ticks,
                        tickDirection * ticks, tickDirection * ticks,
                        power);
                currentDistance = ks109.getDistance();
            }
            telemetry.addLine(String.format("\nDistance=%.2f inch", currentDistance));
            telemetry.update();
            diff = Math.abs(currentDistance - targetDistanceInch);
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

    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle(){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180){
            deltaAngle -= 360;
        }
        else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro",orientation.firstAngle);
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();
        double error = degrees;

        while (opModeIsActive() && Math.abs(error)>2){
            double motorPower = (error < 0 ? -0.3 : 0.3);
            driveWithoutEncoders(-motorPower,-motorPower,motorPower,motorPower);
            error = degrees - getAngle();
            telemetry.addData("error",error);
            telemetry.update();
        }
        driveWithoutEncoders(0,0,0,0);
        for (DcMotor motor:motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void turnTo(double degrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;

        if(error > 180){
            error -= 360;
        }
        else if(error < -180){
            error += 360;
        }
        turn(error);
    }
    public void placeCone(int target, int end, int intervalMs, double wheelPower, double armPower){
        // Raise arm
        driveArm(target, armPower);
        sleep(intervalMs);

        // move forward for x inches
        int forwardPos = 26;
        driveMotors(forwardPos, forwardPos, forwardPos, forwardPos, wheelPower);
        sleep(intervalMs);

        // tilt the grabber
        tilt.setPower(0.5);
        sleep(intervalMs);
        tilt.setPower(0);

        // loose the grabber
        grab.setPower(0.5);
        sleep(intervalMs);

        // driver back
        driveMotors(-forwardPos, -forwardPos, -forwardPos, -forwardPos, wheelPower);

        // close the grabber
        grab.setPower(-0.5);
        sleep(intervalMs);
        grab.setPower(0);

        // Lower arm to the target position
        driveArm(end, armPower);
        sleep(intervalMs);

        // open the grabber
        grab.setPower(0.5);
        sleep(intervalMs);

    }
}
