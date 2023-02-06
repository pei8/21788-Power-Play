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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.a){
                robot.setGrabServoPosition(1);
            }

            if(gamepad1.b){
                robot.setGrabServoPosition(0.5);
            }

            if(gamepad1.x){
                robot.setGrabServoPosition(0);
            }

            telemetry.update();
        }
    }
}
