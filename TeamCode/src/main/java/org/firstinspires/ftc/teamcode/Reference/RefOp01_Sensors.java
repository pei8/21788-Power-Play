package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.driver.KS109I2cDistance;

@TeleOp(name = "RefOp01_Sensors",group = "Linear Opmode")

public class RefOp01_Sensors extends LinearOpMode {
    private DcMotor motorfl;
    private DcMotor motorbl;
    private DcMotor motorfr;
    private DcMotor motorbr;

    private DcMotor armR;
    private DcMotor armL;

    private CRServo grab;
    private CRServo tilt;

    private KS109I2cDistance ks109;  // Hardware Device Object
    //private ColorSensor colorSens;
    private DistanceSensor distanceSens;

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
        ks109 = hardwareMap.get(KS109I2cDistance.class, "ks109");
        //colorSens = hardwareMap.get(ColorSensor.class, "colorSens");
        distanceSens = hardwareMap.get(DistanceSensor.class, "distanceSens");

        motorfr.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);

        armR.setDirection(DcMotor.Direction.REVERSE);

        // Put initialization blocks here.
        int R, G, B, maxVal;
        String color = "Unknown";
        double distanceKs, distanceRev;

        waitForStart();
        while (opModeIsActive()) {
//
//            R = colorSens.red();
//            G = colorSens.green();
//            B = colorSens.blue();
//            maxVal = Math.max(Math.max(R, G), B);
//            if (maxVal >= 200) {
//                if (R == maxVal)
//                    color = "Red";
//                else if (G == maxVal)
//                    color = "Green";
//                else
//                    color = "Blue";
//            }
//            else
//                color = "Unknown";
            distanceKs = ks109.getDistance();
            distanceRev = distanceSens.getDistance(DistanceUnit.INCH);


            telemetry.addLine(String.format("Distance109=%.2f inch\nDistanceRev=%.2f inch",
                    distanceKs, distanceRev));
            telemetry.update();
            sleep(100);
        }
    }
}
