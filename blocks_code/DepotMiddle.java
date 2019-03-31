package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "DepotMiddle3 (Blocks to Java)", group = "")
public class DepotMiddle3 extends LinearOpMode {

  private DcMotor front_right;
  private DcMotor front_left;
  private DcMotor back_right;
  private DcMotor back_left;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double i;

    front_right = hardwareMap.dcMotor.get("front_right");
    front_left = hardwareMap.dcMotor.get("front_left");
    back_right = hardwareMap.dcMotor.get("back_right");
    back_left = hardwareMap.dcMotor.get("back_left");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      for (i = 1; i >= 0; i -= 0.2) {
        front_right.setPower(i * -1);
        front_left.setPower(i);
        back_right.setPower(i * -1);
        back_left.setPower(i);
        sleep(275);
      }
      // Rest
      front_right.setPower(0);
      front_left.setPower(0);
      back_right.setPower(0);
      back_left.setPower(0);
      sleep(500);
      // Going Backward
      for (i = 1; i >= 0; i -= 0.2) {
        front_right.setPower(i);
        front_left.setPower(i * -1);
        back_right.setPower(i);
        back_left.setPower(i * -1);
        sleep(250);
      }
      // 90 Degree Turn
      front_right.setPower(0);
      front_left.setPower(0.5);
      back_right.setPower(0);
      back_left.setPower(0.5);
      sleep(2150);
      // Go Forward
      front_right.setPower(0.5);
      front_left.setPower(-0.5);
      back_right.setPower(-0.5);
      back_left.setPower(-0.5);
      sleep(1450);
      // 45 Degree Turn
      front_right.setPower(0.75);
      front_left.setPower(0);
      back_right.setPower(0.75);
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
    }
  }
}
