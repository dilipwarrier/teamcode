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

import android.graphics.Color;

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


//ALL MEASUREMENTS IN INCHES


@Autonomous(name="SkyStoneRight", group="Linear Opmode")
public class SkyStoneRight extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    double startangle, angle;
    double x, y, fieldwidth, fieldlength;
    double skystoneColorThreshold, distanceBeforeBlocks, inchesStrafePerSec, degreesTurnPerSec, inchesAdvancePerSec, edgeToBlocks, robotWidth, robotLength;
    Servo armservo;
    DcMotor armmotor, armmotor2;
    ColorSensor color_sensor;
    DistanceSensor dist_sensor;
    BNO055IMU imu;

    @Override
    public void runOpMode() {


        //Constant initializations
        skystoneColorThreshold = 0;
        distanceBeforeBlocks = 12;
        inchesStrafePerSec = 12;
        degreesTurnPerSec = 80;
        inchesAdvancePerSec = 31;
        edgeToBlocks = 48;
        robotLength = 18;
        robotWidth = 16;


        //Coordinate reference frame initialization
        angle = 180;
        startangle = angle;
        x = 144 - robotLength / 2;
        y = 108;
        fieldwidth = 144;
        fieldlength = 144;

        //Sensor and motor initializations
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        backRight = hardwareMap.get(DcMotor.class, "back_right");
        color_sensor = hardwareMap.get(ColorSensor.class,"clr");
        armservo = hardwareMap.get(Servo.class, "arm_servo");
        armmotor = hardwareMap.get(DcMotor.class, "arm_motor");
        armmotor2 = hardwareMap.get(DcMotor.class, "arm_motor2");
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
            armservo.setPosition(0);
            double distance = dist_sensor.getDistance(DistanceUnit.INCH);
            dirForward();
            //Need to stop robot before the blocks
            while(distance > distanceBeforeBlocks) {
                setPower(1);
                distance = dist_sensor.getDistance(DistanceUnit.INCH);
            }
            setPower(0);
            turn(0, 0);
            x = 144 - (edgeToBlocks - robotLength / 2 - 6);
            strafeAmount(12, 1);
            boolean skystonefound = false;
            //Strafe until detect change in color from yellow to black(skystone color)
            double red = color_sensor.red();
            double green = color_sensor.green();
            double blue = color_sensor.blue();
            double distanceTrav = 0;
            double starttime = System.currentTimeMillis();
            //Does a constant strafe in along the blocks until detects color change. When detected
            //skystonefound becomes true and it strafes until "green <= skystoneColorThreshold,"
            //or basically until the color changes back to yellow so that the robot will be in the
            //correct position to grab the block
            while(!skystonefound || green <= skystoneColorThreshold) {
                setPower(0.25);
                dirStrafeLeft();
                red = color_sensor.red();
                green = color_sensor.green();
                blue = color_sensor.blue();
                distanceTrav = (System.currentTimeMillis() - starttime) / 1000 * inchesStrafePerSec / 4;
                if(!skystonefound) {
                    //We used the rgb values from color sensor to distinguish skystone color
                    if (green <= skystoneColorThreshold) {
                        skystonefound = true;
                        telemetry.addData("Skystone found", 0);
                        telemetry.update();
                    }
                }
                telemetry.addData("Green", green);
                telemetry.update();
            }
            setPower(0);
            if(skystonefound) {
                telemetry.addData("Ready to grab", 0);
                telemetry.update();
                y += distanceTrav;
            }
            armDown();
            sleep(300);
            armservo.setPosition(1);
            turn(180, 0);
            backadvance(x - 12);
            turn(270, 0);
            advance(y - 72);
        }
    }
    public void armDown() {
        armmotor.setDirection(DcMotor.Direction.REVERSE);
        armmotor2.setDirection(DcMotor.Direction.FORWARD);
        armmotor.setPower(0.5);
        armmotor2.setPower(0.5);
        sleep(300);
        armmotor.setPower(0);
        armmotor2.setPower(0);
    }
    public void armUp() {
        armmotor.setDirection(DcMotor.Direction.FORWARD);
        armmotor2.setDirection(DcMotor.Direction.REVERSE);
        armmotor.setPower(0.5);
        armmotor2.setPower(0.5);
        sleep(700);
        armmotor.setPower(0);
        armmotor2.setPower(0);
    }
    //Strafes inches in certain direction. 0 means right while 1 means left
    public void strafeAmount(double inchesStrafe, int direction) {
        if(direction == 1) {
            dirStrafeLeft();
            y -= inchesStrafe;
        }else {
            dirStrafeRight();
            y += inchesStrafe;
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
    public void turn(double rotToAng, int level) {
        if(level >= 2) {
            return;
        }
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
        setPower(0.25);
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
    //Advance forward distance
    public void advance(double distance) {
        dirForward();
        setPower(1);
        double waitTime = distance / inchesAdvancePerSec * 1000;
        sleep((long)waitTime);
        setPower(0);
    }
    public double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }
    //Advance backwards distance
    public void backadvance(double distance) {
        dirBackward();
        setPower(1);
        double waitTime = distance / inchesAdvancePerSec * 1000;
        sleep((long)waitTime);
        setPower(0);
    }
    //Sets motor power
    public void setPower(double power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    //All dir functions below sets the motor directions

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
    public void dirBackward() {
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }
}
