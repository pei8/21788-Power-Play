package org.firstinspires.ftc.teamcode.common.hardware;


import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@Autonomous(name="Gyro", group = "test")
public class Gyro extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // imu variables
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        while (opModeIsActive()){
//            Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            telemetry.addData("Orientation: ",orientation.firstAngle);
//            telemetry.update();
//        }

        turnTo(90);
        sleep(3000);


    }
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle(){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle>180){
            deltaAngle -= 360;
        }
        else if (deltaAngle<=-180){
            deltaAngle+=360;
        }

        currAngle+=deltaAngle;
        lastAngles = orientation;

        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error)>2){
            //double motorPower = (error < 0 ? -0.3 : 0.3);
            double motorPower = Math.min(error*0.01,0.3);
            robot.setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setAllDrivePower(0);
    }
    public void turnTo(double degrees){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180){
            error -= 360;
        }
        else if (error < -180) {
            error += 360;
        }
        while (opModeIsActive() && Math.abs(error)>10){
            Orientation currOrientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double motorPower = (error < 0 ? -0.35 : 0.35);
            robot.setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - currOrientation.firstAngle;
            telemetry.addData("Orientation: ",currOrientation.firstAngle);
            telemetry.addData("Error: ",Math.abs(error));

            telemetry.update();

        }

    }
}
