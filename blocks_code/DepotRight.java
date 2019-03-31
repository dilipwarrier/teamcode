package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DepotRight2 (Blocks to Java)", group = "")
public class DepotRight2 extends LinearOpMode {

  private DcMotor front_left;
  private DcMotor front_right;
  private DcMotor back_left;
  private DcMotor back_right;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    front_left = hardwareMap.dcMotor.get("front_left");
    front_right = hardwareMap.dcMotor.get("front_right");
    back_left = hardwareMap.dcMotor.get("back_left");
    back_right = hardwareMap.dcMotor.get("back_right");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      // Going to knock off the right most block
      front_left.setPower(0.4);
      front_right.setPower(0);
      back_left.setPower(0.4);
      back_right.setPower(0);
      sleep(1000);
      // The turn was above, going forward below
      front_left.setPower(0.5);
      front_right.setPower(-0.5);
      back_right.setPower(-0.5);
      back_left.setPower(0.5);
      sleep(1700);
      // Going back (forward)
      front_left.setPower(-0.5);
      front_right.setPower(0.5);
      back_left.setPower(-0.5);
      back_right.setPower(0.5);
      sleep(1500);
      // Going back (turn)
      front_left.setPower(-0.3);
      front_right.setPower(0);
      back_left.setPower(-0.3);
      back_right.setPower(0);
      sleep(1600);
      front_left.setPower(0);
      front_right.setPower(0);
      back_left.setPower(0);
      back_right.setPower(0);
      sleep(150);
      front_left.setPower(0.5);
      front_right.setPower(-0.5);
      back_left.setPower(0.5);
      back_right.setPower(-0.5);
      sleep(500);
      // 90 turn
      front_right.setPower(0);
      front_left.setPower(0.5);
      back_right.setPower(0);
      back_left.setPower(0.5);
      sleep(2150);
      // Forward
      front_right.setPower(1);
      front_left.setPower(-1);
      back_right.setPower(1);
      back_left.setPower(-1);
      sleep(700);
      // 45 turn
      front_right.setPower(0.5);
      front_left.setPower(0);
      back_right.setPower(0.5);
      back_left.setPower(0);
      sleep(500);
      front_right.setPower(0.5);
      front_left.setPower(0);
      back_right.setPower(0.5);
      back_left.setPower(0);
      sleep(1500);
      front_right.setPower(1);
      front_left.setPower(-1);
      back_right.setPower(1);
      back_left.setPower(-1);
      sleep(1000);
    }
  }
}
