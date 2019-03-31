package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "CraterLeft4 (Blocks to Java)", group = "")
public class CraterLeft4 extends LinearOpMode {

  private DcMotor front_right;
  private DcMotor back_right;
  private DcMotor front_left;
  private DcMotor back_left;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    front_right = hardwareMap.dcMotor.get("front_right");
    back_right = hardwareMap.dcMotor.get("back_right");
    front_left = hardwareMap.dcMotor.get("front_left");
    back_left = hardwareMap.dcMotor.get("back_left");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Moving to the left
        front_right.setPower(-0.4);
        back_right.setPower(-0.4);
        front_left.setPower(0);
        back_left.setPower(0);
        sleep(1000);
        // Move forward to move off cube
        front_left.setPower(0.5);
        front_right.setPower(-0.5);
        back_right.setPower(-0.5);
        back_left.setPower(0.5);
        sleep(1500);
        // Move backward to start
        front_right.setPower(0.4);
        back_right.setPower(0.4);
        front_left.setPower(-0.4);
        back_left.setPower(-0.4);
        sleep(1250);
        // Moving to the right
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0.5);
        back_right.setPower(0.5);
        sleep(1000);
        // 90 Degree Turn
        front_right.setPower(-0.5);
        front_left.setPower(0);
        back_right.setPower(-0.5);
        back_left.setPower(0);
        sleep(2500);
        // Go Forward
        front_right.setPower(-0.5);
        front_left.setPower(0.5);
        back_right.setPower(-0.5);
        back_left.setPower(0.5);
        sleep(1000);
        // 45 Degree Turn
        front_right.setPower(-0.75);
        front_left.setPower(0);
        back_right.setPower(-0.75);
        back_left.setPower(0);
        sleep(1000);
        // Go Forward
        front_right.setPower(-0.5);
        front_left.setPower(0.5);
        back_right.setPower(-0.5);
        back_left.setPower(0.5);
        sleep(2100);
        front_right.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
        sleep(1000);
        front_right.setPower(0.75);
        front_left.setPower(-0.75);
        back_right.setPower(0.75);
        back_left.setPower(-0.75);
        sleep(5000);
        break;
      }
    }
  }
}
