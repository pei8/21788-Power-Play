package org.firstinspires.ftc.teamcode.jakecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="JakeAuto", group="auto")
public class JakeAuto extends LinearOpMode
{

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
    public void runOpMode()
    {

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
        }
        for (DcMotor arm: arms){
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            grab.setPower(-0.5);

            sleep(2000);
            driveMotors(250,250,250,250,0.25);
            sleep(1000);
            driveMotors(-1,-1,-1,-1,0.25);

            if(tagOfInterest.id == LEFT){
                sleep(1000);
                driveMotors(-200,-200,200,200,0.5);
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

    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget, double speed){
        flPos += flTarget;
        blPos += blTarget;
        frPos += frTarget;
        brPos += brTarget;


        for (DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        motorfl.setTargetPosition(flPos);
        motorbl.setTargetPosition(blPos);
        motorfr.setTargetPosition(frPos);
        motorbr.setTargetPosition(brPos);

        for (DcMotor motor: motors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(speed);
        }

        while(opModeIsActive() && motorfl.isBusy() && motorbl.isBusy() && motorfr.isBusy() && motorbr.isBusy()){
            idle();
        }
        for (DcMotor motor: motors){
            motor.setPower(0);
        }
    }


    private void driveArm(int armTarget, double speed){
        armPos += armTarget;

        for (DcMotor arm: arms){
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setTargetPosition(armPos);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(speed);
        }
        while(opModeIsActive() && armR.isBusy() && armL.isBusy()){
            idle();
        }
        for (DcMotor arm: arms){
            arm.setPower(0);
        }
    }

}
