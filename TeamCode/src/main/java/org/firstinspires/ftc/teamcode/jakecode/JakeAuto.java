package org.firstinspires.ftc.teamcode.jakecode;

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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="JakeAuto", group="auto")
public class JakeAuto extends LinearOpMode {

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

    // Tag id
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

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

    private int flPos;
    private int blPos;
    private int frPos;
    private int brPos;

    private int armPos;

    private List<DcMotor> motors;
    private List<DcMotor> arms;


    @Override
    public void runOpMode() {
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

        motors = Arrays.asList(motorfl,motorbl,motorbr,motorfr); //array containing all motors
        arms = Arrays.asList(armL,armR);//array of arm motors

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
        aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.jakecode.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_RIGHT);}
            @Override public void onError(int errorCode) {}});
        telemetry.setMsTransmissionInterval(50);
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if(currentDetections.size() != 0) {
                for(AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        telemetry.addLine(String.format("\nDetected tag ID=%d",tagOfInterest.id));
                        break;}}}
            telemetry.update();
            sleep(20);}



        // The program is started
        if (isStarted()){
            //test code

            grab.setPower(-0.5);

            //push signal cone away
            sleep(1000);
            driveMotors(400,400,400,400,0.25);
            sleep(1000);
            driveMotors(-60,-60,-60,-60,0.25);
            sleep(1000);

            //turn right
            driveMotors(100,100,-100,-100,0.25);
            sleep(1000);

            //place cone
            driveArm(0,1);
            sleep(4000);


            requestOpModeStop();


            placeCone(10,17);




            driveMotors(-320,-320,320,320,0.25);
            sleep(1000);

            grab.setPower(0.5);
            sleep(2000);

            driveMotors(150,150,150,150,0.25);
            sleep(1000);

            grab.setPower(-0.5);
            sleep(2000);

            driveArm(0,1);
            sleep(2000);

            driveMotors(-100,-100,-100,-100,0.25);
            sleep(2000);



            if(tagOfInterest.id == LEFT){
                sleep(1000);
                driveMotors(-200,-200,200,200,0.25);
                sleep(1000);
                driveMotors(70,70,70,70,0.25);
            }
            else if(tagOfInterest.id == MIDDLE) {
                driveMotors(70,70,70,70,0.25);
            }
            else if (tagOfInterest.id == RIGHT) {
                driveMotors(200, 200, -200, -200, 0.25);
                driveMotors(70, 70, 70, 70, 0.25);
            }

        }
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


        for (DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        for (DcMotor motor: motors){
            motor.setPower(speed);
        }

        while(opModeIsActive() && motorfl.isBusy() && motorbl.isBusy() && motorfr.isBusy() && motorbr.isBusy()){
            idle();
        }
        for (DcMotor motor: motors){
            motor.setPower(0);
        }
    }
    private void driveArm(int armTarget, double power){

        armPos = armTarget;
        telemetry.addLine(String.valueOf(armPos));
        for (DcMotor arm: arms){
            //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(armPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(power);
        }

        telemetry.update();
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
    public void placeCone(int target, int end){
        driveArm(target,0.2);
        sleep(1000);

        driveMotors(40,40,40,40,0.25);
        sleep(1000);

        tilt.setPower(0.5);
        sleep(2000);
        grab.setPower(0.5);
        sleep(2000);
        grab.setPower(-0.5);
        sleep(2000);

        driveArm(end,0.3);
        sleep(2000);

        grab.setPower(0);
        sleep(2000);
    }
}
