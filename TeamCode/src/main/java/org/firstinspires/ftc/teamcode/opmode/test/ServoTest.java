package org.firstinspires.ftc.teamcode.opmode.test;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name="ServoTest")
public class ServoTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        double tiltPos = 0.5;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.right_trigger>0.5){
                robot.grabServo.setPosition(0.4); // open
            }
            else if (gamepad2.left_trigger>0.5){
                robot.grabServo.setPosition(1); // close
            }


            //robot.tiltServo.setPosition((gamepad2.left_stick_y+1)/2);


            if (gamepad2.right_bumper){
                tiltPos = Math.min(tiltPos + 0.05,1);
                robot.tiltServo.setPosition(tiltPos);
                while(gamepad2.right_bumper){
                    idle();
                }
            }

            if (gamepad2.left_bumper){
                tiltPos = Math.max(tiltPos - 0.05,0);
                robot.tiltServo.setPosition(tiltPos); // close
                while(gamepad2.left_bumper){
                    idle();
                }
            }

            telemetry.addData("Tilt Servo Position: ",String.valueOf(robot.tiltServo.getPosition()));
            telemetry.addData("TiltPos Var: ",tiltPos);

            telemetry.update();
        }
    }
}
