package org.firstinspires.ftc.teamcode.jakecode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "JakeOp",group = "Linear Opmode")

public class JakeOp extends LinearOpMode {

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

        motorfr.setDirection(DcMotor.Direction.REVERSE);

        armR.setDirection(DcMotor.Direction.REVERSE);



        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 0.5;
            double vertical = -gamepad1.left_stick_y * 0.5;
            double turn = gamepad1.right_stick_x * 0.5;

            motorfl.setPower(vertical+turn-horizontal);
            motorbl.setPower(vertical+turn+horizontal);
            motorfr.setPower(vertical-turn+horizontal);
            motorbr.setPower(vertical-turn-horizontal);

            double rightArm = gamepad2.left_stick_y;
            double leftArm = gamepad2.left_stick_y;

            armR.setPower(rightArm * 0.8);
            armL.setPower(leftArm * 0.8);

            double tiltServo = gamepad2.right_stick_y * 0.5;
            double grabServo = gamepad2.right_trigger - gamepad2.left_trigger;
            grab.setPower(grabServo);
            tilt.setPower(tiltServo);
            telemetry.addLine(String.valueOf(tiltServo));

            telemetry.update();
        }
    }
}