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

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 0.45;
            double vertical = -gamepad1.left_stick_y * 0.45;
            double turn = gamepad1.right_stick_x * 0.45;

            robot.setDrivePower(vertical+turn-horizontal,vertical-turn+horizontal,vertical+turn+horizontal,vertical-turn-horizontal);

            if (-gamepad2.left_stick_y > 0.1){
                robot.setArmPower(0.8);
            }
            else if (-gamepad2.left_stick_y < -0.1){
                robot.setArmPower(-0.8);
            }
            else{
                robot.setArmPower(0);
            }

            // tilt 0.7 is highest junction tilt
            robot.tiltServo.setPosition(Math.max(Math.min((gamepad2.right_stick_y+1)/2, 0.8),0.55));

            if (gamepad2.right_trigger>0.5){
                robot.grabServo.setPosition(0.4); // open
            }
            else if (gamepad2.left_trigger>0.5){
                robot.grabServo.setPosition(1); // close
            }


            telemetry.update();
        }
    }
}