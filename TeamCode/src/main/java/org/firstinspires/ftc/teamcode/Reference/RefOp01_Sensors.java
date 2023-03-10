package org.firstinspires.ftc.teamcode.Reference;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.hardware.KS109I2cDistance;

@TeleOp(name = "RefOp01_Sensors",group = "Linear Opmode")

public class RefOp01_Sensors extends LinearOpMode {
    IMU imu;
    private DcMotor motorfl;
    private DcMotor motorbl;
    private DcMotor motorfr;
    private DcMotor motorbr;

    private DcMotor armR;
    private DcMotor armL;

    private Servo grab;
    private Servo tilt;

    private KS109I2cDistance ks109;  // Hardware Device Object
    //private ColorSensor colorSens;
    private DistanceSensor distanceSens, distanceSens2, distanceFront;
    // Initial robot orientation
    YawPitchRollAngles orientation0;
    AngularVelocity angularVelocity0;
    double yaw0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
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
        //colorSens = hardwareMap.get(ColorSensor.class, "colorSens");
        distanceSens = hardwareMap.get(DistanceSensor.class, "distanceSens");
        distanceSens2 = hardwareMap.get(DistanceSensor.class, "distanceSens2");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");

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


            telemetry.addLine(String.format("Distance109=%.2f inch\nDistanceRev1=%.2f inch\nDistanceRev2=%.2f inch\nDistanceRevFront=%.2f inch\nYaw=%.2f Degree",
                    ks109.getDistance(),
                    distanceSens.getDistance(DistanceUnit.INCH),
                    distanceSens2.getDistance(DistanceUnit.INCH),
                    distanceFront.getDistance(DistanceUnit.INCH),
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
                    )
            );
            telemetry.update();
            sleep(100);
        }
    }
}
