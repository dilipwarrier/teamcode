package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Motormover (Blocks to Java)", group = "")
public class Motormover extends LinearOpMode {

  private DcMotor ball_collector;
  private DcMotor ball_collector1;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    ball_collector = hardwareMap.dcMotor.get("ball_collector");
    ball_collector1 = hardwareMap.dcMotor.get("ball_collector1");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        ball_collector.setPower(0.4);
        ball_collector1.setPower(-0.4);
        sleep(2000);
        ball_collector.setPower(0);
        ball_collector1.setPower(0);
        sleep(2500);
        ball_collector.setPower(-0.6);
        ball_collector1.setPower(0.6);
        sleep(1750);
      }
    }
  }
}
