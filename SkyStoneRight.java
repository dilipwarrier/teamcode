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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.files.DataLogger;
import java.util.logging.Logger;

@Autonomous(name="SkyStoneRight", group="Linear Opmode")
public class SkyStoneRight extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    Servo leftClip = null;
    Servo rightClip = null;
    //DcMotor vert_motor = null;
    //DcMotor horz_motor = null;
    //Servo armservo = null;
    double startangle, angle;
    double x, y, fieldwidth, fieldlength;
    double skystoneColorThreshold, distanceBeforeBlocks, inchesStrafePerSec, degreesTurnPerSec, inchesAdvancePerSec, edgeToBlocks, robotWidth, robotLength;
    ColorSensor color_sensor;
    DistanceSensor dist_sensor;
    BNO055IMU imu;

    @Override
    public void runOpMode() {


        //Constant initializations
        skystoneColorThreshold = 1;
        distanceBeforeBlocks = 5;
        inchesStrafePerSec = 12;
        degreesTurnPerSec = 80;
        inchesAdvancePerSec = 14;
        edgeToBlocks = 48;
        robotLength = 18;
        robotWidth = 16;


        //Coordinate reference frame initialization
        angle = 180;
        startangle = angle;
        fieldwidth = 144;
        fieldlength = 144;
        x = fieldwidth - robotLength / 2;
        y = 104;

        //Sensor and motor initializations
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        color_sensor = hardwareMap.get(ColorSensor.class,"clr");
        leftClip = hardwareMap.get(Servo.class, "left_clip");
        rightClip = hardwareMap.get(Servo.class, "right_clip");
        //vert_motor = hardwareMap.get(DcMotor.class, "vert_motor");
        //horz_motor = hardwareMap.get(DcMotor.class, "horz_motor");
        //armservo = hardwareMap.get(Servo.class, "arm_servo");
        dist_sensor = hardwareMap.get(DistanceSensor.class, "distance");


        //Gyro initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        if(opModeIsActive()) {
            leftClip.setPosition(1);
            rightClip.setPosition(0);
            double distance = dist_sensor.getDistance(DistanceUnit.INCH);
            dirForward();
            while(distance > distanceBeforeBlocks) {
                setPower(0.5);
                distance = dist_sensor.getDistance(DistanceUnit.INCH);
            }
            setPower(0);
            turn(startangle, 0);


            //Because coordinates are of center of robot, to have front of robot 3 inches away
            //from 48 inches (where the blocks are), it must be at 48 - 8 (half of robot) - 3 = 37 inches
            x = fieldwidth - edgeToBlocks + robotLength / 2 + 3;
            boolean skystonefound = false;
            int counter = 0;

            //Strafe until detect change in color from yellow to black(skystone color)
            while(!skystonefound) {
                if(counter >= 4){
                    skystonefound = true;
                    break;
                }
                sleep(1500);
                //We used the red value from color sensor to distinguish skystone color
                double red = color_sensor.red();
                double green = color_sensor.green();
                telemetry.update();
                if(Math.abs(red - green) <= skystoneColorThreshold) {
                    skystonefound = true;
                    break;
                }
                strafeAmount(8, 1);
                counter++;
            }
            telemetry.addData("Status:", "Skystone found");
            telemetry.update();
            if(skystonefound) {
                strafeAmount(4, 0);
                advance(18);
                x -= 18;
                leftClip.setPosition(0);
                rightClip.setPosition(1);
                sleep(1000);
            }
            moveTo(108, y);
            moveTo(108, 65);
            turn(270, 0);
        }
    }

    //Strafes inches in certain direction. 0 means right while 1 means left
    public void strafeAmount(double inchesStrafe, int direction) {
        if(direction == 1) {
            dirStrafeLeft();
            y += inchesStrafe;
        }else {
            dirStrafeRight();
            y -= inchesStrafe;
        }
        setPower(0.5);
        double timeWait = inchesStrafe / inchesStrafePerSec * 1000;
        sleep((long)timeWait);
        setPower(0);
    }

    //Move robot to (destx, desty)
    public void moveTo(double destx, double desty) {
        double xdif = destx - x;
        double ydif = desty - y;
        double angRot = Math.toDegrees(Math.atan(ydif/xdif));
        if(destx < x) {
            angRot += 180;
        }
        if(angRot < 0){
            angRot += 360;
        }else if(angRot >= 360) {
            angRot -= 360;
        }
        telemetry.addData("Degree of Rotation", "{%.2f}", angRot);
        telemetry.update();
        turn(angRot, 0);
        advance(distance(x, y, destx, desty));
        x = destx;
        y = desty;
    }

    //Turn to rotToAng
    //Turn to rotToAng
    public void turn(double rotToAng, int level) {
        if(level >= 2) {
            return;
        }
        setPower(0.5);
        double angDif = rotToAng - angle;
        if (angDif < 0) {
            angDif += 360;
        }
        if (angDif > 180) {
            angDif -= 360;
            angDif *= -1;
            dirLeft();
        } else {
            dirRight();
        }
        setPower(0.5);
        double waitTime = angDif / degreesTurnPerSec * 1000;
        angle = rotToAng;
        sleep((long)waitTime);
        setPower(0);
        sleep(1000);
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        float actualAngle = angles.firstAngle;
        actualAngle = 360 - actualAngle;
        actualAngle += startangle;
        if(actualAngle >= 360){
            actualAngle -= 360;
        }
        double correction = Math.abs(actualAngle - rotToAng);
        angle = actualAngle;
        if(correction < 5 || correction > 355) {
            return;
        }else{
            turn(rotToAng, level + 1);
        }
    }
    public void advance(double distance) {
        dirForward();
        setPower(0.5);
        double waitTime = distance / inchesAdvancePerSec * 1000;
        sleep((long)waitTime);
        setPower(0);
    }
    public void setPower(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
    public void dirLeft() {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    public void dirRight() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
    public void dirForward() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    public void dirStrafeRight() {
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    public void dirStrafeLeft() {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
    public double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    public void backadvance(double distance) {
        dirBackward();
        setPower(0.5);
        double waitTime = distance / inchesAdvancePerSec * 1000;
        sleep((long)waitTime);
        setPower(0);
    }
    public void dirBackward() {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
}
