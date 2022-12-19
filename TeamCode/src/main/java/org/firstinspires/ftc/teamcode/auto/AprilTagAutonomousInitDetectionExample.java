package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
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
    @Override
    public void runOpMode()
    {

        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");

        motors = Arrays.asList(motorfl,motorbl,motorbr,motorfr);

        armR = hardwareMap.get(DcMotor.class, "armR");
        armL = hardwareMap.get(DcMotor.class, "armL");

        grab = hardwareMap.get(CRServo.class, "grab");
        tilt = hardwareMap.get(CRServo.class, "tilt");


        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfr.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);

        motorfl.setTargetPosition(0);
        motorbl.setTargetPosition(0);
        motorfr.setTargetPosition(0);
        motorbr.setTargetPosition(0);

        motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armL.setDirection(DcMotor.Direction.REVERSE);

        armR.setTargetPosition(0);
        armL.setTargetPosition(0);

        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flPos = 0;
        blPos = 0;
        frPos = 0;
        brPos = 0;

        armPos = 0;


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new org.firstinspires.ftc.teamcode.auto.AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        telemetry.addLine(String.format("\nDetected tag ID=%d",tagOfInterest.id));
                        break;
                    }
                }
            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Actually do something useful */
        if (isStarted()){
            grab.setPower(0.5);
            sleep(2000);
            drive(250,250,250,250,0.25);
            sleep(1000);
            drive(-1,-1,-1,-1,0.25);
            if(tagOfInterest.id == LEFT){
                sleep(1000);
                drive(-200,-200,200,200,0.5);
                sleep(1000);
                drive(70,70,70,70,0.25);
            }
            else if(tagOfInterest == null||tagOfInterest.id == MIDDLE) {
                drive(70,70,70,70,0.25);
            }else{
                drive(200,200,-200,-200,0.25);
                drive(70,70,70,70,0.25);
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }
    private void drive(int flTarget, int blTarget, int frTarget, int brTarget, double speed){
        flPos += flTarget;
        blPos += blTarget;
        frPos += frTarget;
        brPos += brTarget;

        telemetry.addLine(String.valueOf(flPos));
        telemetry.addLine(String.valueOf(frPos));
        telemetry.addLine(String.valueOf(blPos));
        telemetry.addLine(String.valueOf(brPos));

        telemetry.update();

        motorfl.setTargetPosition(flPos);
        motorbl.setTargetPosition(blPos);
        motorfr.setTargetPosition(frPos);
        motorbr.setTargetPosition(brPos);

        motorfl.setPower(speed);
        motorbl.setPower(speed);
        motorfr.setPower(speed);
        motorbr.setPower(speed);

        while(opModeIsActive() && motorfl.isBusy() && motorbl.isBusy() && motorfr.isBusy() && motorbr.isBusy()){
            idle();
        }
        motorfl.setPower(0);
        motorbl.setPower(0);
        motorfr.setPower(0);
        motorbr.setPower(0);
    }


    private void moveArm(int armTarget, double speed){
        armPos += armTarget;

        armR.setTargetPosition(armPos);
        armL.setTargetPosition(armPos);

        armR.setPower(speed);
        armL.setPower(speed);

        while(opModeIsActive() && armR.isBusy() && armL.isBusy()){
            idle();
        }
    }

}
