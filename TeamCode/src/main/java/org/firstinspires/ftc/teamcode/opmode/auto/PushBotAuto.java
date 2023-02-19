/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
/**
 * Encoder-based move in square with both turn and sidestep movement
 */

@Autonomous(name="PushBotAuto")

public class PushBotAuto extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot   = new RobotHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 80 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    static final double     ARM_SPEED               = 0.4;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


        robot.setDrivetrainMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.setArmsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // drive in square with sidestep
        //encoderDrive(DRIVE_SPEED, 10, 10, 10, 10);


        sleep(3000);

        encoderArm(ARM_SPEED, 50);

        sleep(3000);

        encoderArm(ARM_SPEED, 65);

        sleep(3000);

        encoderArm(ARM_SPEED, 0);

        sleep(3000);

        requestOpModeStop();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double leftFront, double rightFront, double leftBack, double rightBack) {
        int newLFTarget;
        int newRFTarget;
        int newLBTarget;
        int newRBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLFTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (leftFront * COUNTS_PER_INCH);
            newRFTarget = robot.frontRightDrive.getCurrentPosition() + (int) (rightFront * COUNTS_PER_INCH);
            newLBTarget = robot.backLeftDrive.getCurrentPosition() + (int) (leftBack * COUNTS_PER_INCH);
            newRBTarget = robot.backRightDrive.getCurrentPosition() + (int) (rightBack * COUNTS_PER_INCH);

            robot.frontLeftDrive.setTargetPosition(newLFTarget);
            robot.frontRightDrive.setTargetPosition(newRFTarget);
            robot.backLeftDrive.setTargetPosition(newLBTarget);
            robot.backRightDrive.setTargetPosition(newRBTarget);

            // Turn On RUN_TO_POSITION
            robot.setDrivetrainMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy()
                            && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {
            }

            // Stop all motion;
            robot.setAllDrivePower(0);

            // Turn off RUN_TO_POSITION
            robot.setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
    }

        public void encoderArm ( double speed, int armPos){
            //robot.setArmsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller

                robot.armLeftMotor.setTargetPosition(armPos);
                robot.armRightMotor.setTargetPosition(armPos);

                // Turn On RUN_TO_POSITION
                robot.setArmsMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                robot.armRightMotor.setPower(Math.abs(speed));
                robot.armLeftMotor.setPower(Math.abs(speed));


                while (opModeIsActive() && (robot.armRightMotor.isBusy() && robot.armLeftMotor.isBusy())) {
                    telemetry.addData("armRight Pos:",robot.armRightMotor.getCurrentPosition());
                    telemetry.addData("armLeft Pos:",robot.armLeftMotor.getCurrentPosition());
                    telemetry.update();
                    idle();
                }



            }
        }
}
