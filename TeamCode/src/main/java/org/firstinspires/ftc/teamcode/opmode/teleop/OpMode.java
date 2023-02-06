package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;


@TeleOp(name = "OpMode")

public class OpMode extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 0.5;
            double vertical = -gamepad1.left_stick_y * 0.5;
            double turn = gamepad1.right_stick_x * 0.5;

            robot.setDrivePower(vertical+turn-horizontal,vertical-turn+horizontal,vertical+turn+horizontal,vertical-turn-horizontal);

            robot.setArmPower(-gamepad2.left_stick_y);

            double tiltServo = gamepad2.right_stick_y * 0.5;


            if (gamepad2.right_bumper){
                robot.setGrabServoPosition(0.3); // open
            }
            else if (gamepad2.left_bumper){
                robot.setGrabServoPosition(1); // close
            }


            telemetry.update();
        }
    }
}