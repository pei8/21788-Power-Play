package org.firstinspires.ftc.teamcode.Reference;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "RefOp01_Color",group = "Linear Opmode")

public class RefOp01_Color extends LinearOpMode {
    private ColorSensor colorSens;
    private DistanceSensor distanceSens;

    private DcMotor motorfl;
    private DcMotor motorbl;
    private DcMotor motorfr;
    private DcMotor motorbr;

    private DcMotor armR;
    private DcMotor armL;

    private CRServo grab;
    private CRServo tilt;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {


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

        // Put initialization blocks here.
        int R, G, B, maxVal;

        waitForStart();
        while (opModeIsActive()) {
            R = colorSens.red();
            G = colorSens.green();
            B = colorSens.blue();
            maxVal = Math.max(Math.max(R, G), B);

            telemetry.addLine(String.format("\nDetected RGB=%d, %d, %d", R, G, B));
            telemetry.update();
            sleep(100);
        }
    }
}
