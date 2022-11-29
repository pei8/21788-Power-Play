package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="JakeAuto", group="auto")
public class JakeAuto extends LinearOpMode {
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

        armR.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();



        motorfl.setPower(0.2);
        motorfr.setPower(0.2);
        motorbl.setPower(0.2);
        motorbr.setPower(0.2);

        while (opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Distance", distanceSens.getDistance(DistanceUnit.CM));
            if (distanceSens.getDistance(DistanceUnit.CM) <= 16) {
                motorfl.setPower(0);
                motorfr.setPower(0);
                motorbl.setPower(0);
                motorbr.setPower(0);
                sleep(1000);
                if (JavaUtil.maxOfList(JavaUtil.createListWith(colorSens.green(), colorSens.blue(), colorSens.red())) == colorSens.red()) {
                    telemetry.addData("red:", "zone 1");
                    motorfl.setPower(-0.5);
                    motorfr.setPower(0.5);
                    motorbl.setPower(-0.5);
                    motorbr.setPower(0.5);
                    sleep(400);
                    motorfl.setPower(0);
                    motorfr.setPower(0);
                    motorbl.setPower(0);
                    motorbr.setPower(0);
                    sleep(500);

                    motorfl.setPower(0.5);
                    motorfr.setPower(0.5);
                    motorbl.setPower(0.5);
                    motorbr.setPower(0.5);
                    sleep(400);
                    motorfl.setPower(0);
                    motorfr.setPower(0);
                    motorbl.setPower(0);
                    motorbr.setPower(0);


                    break;
                }
                if (JavaUtil.maxOfList(JavaUtil.createListWith(colorSens.green(), colorSens.blue(), colorSens.red())) == colorSens.green()) {
                    telemetry.addData("green:", "zone 2");
                    motorfl.setPower(0.5);
                    motorfr.setPower(0.5);
                    motorbl.setPower(0.5);
                    motorbr.setPower(0.5);
                    sleep(200);
                    motorfl.setPower(0);
                    motorfr.setPower(0);
                    motorbl.setPower(0);
                    motorbr.setPower(0);
                    break;
                }
                if (JavaUtil.maxOfList(JavaUtil.createListWith(colorSens.green(), colorSens.blue(), colorSens.red())) == colorSens.blue()) {
                    telemetry.addData("blue:", "zone 3");
                    motorfl.setPower(0.5);
                    motorfr.setPower(-0.5);
                    motorbl.setPower(0.5);
                    motorbr.setPower(-0.5);
                    sleep(450);
                    motorfl.setPower(0);
                    motorfr.setPower(0);
                    motorbl.setPower(0);
                    motorbr.setPower(0);
                    sleep(500);

                    motorfl.setPower(0.5);
                    motorfr.setPower(0.5);
                    motorbl.setPower(0.5);
                    motorbr.setPower(0.5);
                    sleep(400);
                    motorfl.setPower(0);
                    motorfr.setPower(0);
                    motorbl.setPower(0);
                    motorbr.setPower(0);
                    break;
                }
            }
        }
    }
}
