package org.firstinspires.ftc.teamcode.jakecode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Disabled
@Autonomous(name="JakeOldAuto", group="auto")
public class JakeOldAuto extends LinearOpMode {
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
    @Override
    public void runOpMode() throws InterruptedException{
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");

        armR = hardwareMap.get(DcMotor.class, "armR");
        armL = hardwareMap.get(DcMotor.class, "armL");

        grab = hardwareMap.get(CRServo.class, "grab");
        tilt = hardwareMap.get(CRServo.class, "tilt");

        colorSens = hardwareMap.get(ColorSensor.class, "colorSens");
        distanceSens = hardwareMap.get(DistanceSensor.class, "distanceSens");

        motorfr.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);

        armL.setDirection(DcMotor.Direction.REVERSE);

        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        flPos = 0;
        blPos = 0;
        frPos = 0;
        brPos = 0;

        armPos = 0;

        waitForStart();

        moveArm(20,0.25);

        //drive(1000, 1000, 1000,1000,0.25);qq

        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Distance", distanceSens.getDistance(DistanceUnit.CM));
            if (distanceSens.getDistance(DistanceUnit.CM) <= 16) {
                sleep(1000);
                if (JavaUtil.maxOfList(JavaUtil.createListWith(colorSens.green(), colorSens.blue(), colorSens.red())) == colorSens.red()) {
                    telemetry.addData("red:", "zone 1");
                    break;
                }
                if (JavaUtil.maxOfList(JavaUtil.createListWith(colorSens.green(), colorSens.blue(), colorSens.red())) == colorSens.green()) {
                    telemetry.addData("green:", "zone 2");
                    break;
                }
                if (JavaUtil.maxOfList(JavaUtil.createListWith(colorSens.green(), colorSens.blue(), colorSens.red())) == colorSens.blue()) {
                    telemetry.addData("blue:", "zone 3");
                    break;
                }

            }

        }

    }
    private void drive(int flTarget, int blTarget, int frTarget, int brTarget, double speed){
        flPos += flTarget;
        blPos += blTarget;
        frPos += frTarget;
        brPos += brTarget;

        motorfl.setTargetPosition(flPos);
        motorbl.setTargetPosition(blPos);
        motorfr.setTargetPosition(frPos);
        motorbr.setTargetPosition(brPos);

        motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorfl.setPower(speed);
        motorbl.setPower(speed);
        motorfr.setPower(speed);
        motorbr.setPower(speed);

        while(opModeIsActive() && motorfl.isBusy() && motorbl.isBusy() && motorfr.isBusy() && motorbr.isBusy()){
            idle();
        }
    }

    private void moveArm(int armTarget, double speed){
        armPos += armTarget;

        armR.setTargetPosition(armTarget);
        armL.setTargetPosition(armTarget);

        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armR.setPower(speed);
        armL.setPower(speed);

        while(opModeIsActive() && armR.isBusy() && armL.isBusy()){
            idle();
        }
    }
}
