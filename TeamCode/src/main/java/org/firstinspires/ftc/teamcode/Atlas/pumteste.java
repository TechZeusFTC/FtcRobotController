package org.firstinspires.ftc.teamcode.Atlas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="peido", group="Iterative OpMode")
public class pumteste extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Recupera o IMU do mapa de hardware
        //IMU imu = hardwareMap.get(IMU.class, "imu");
        //Ajuste os parâmetros de orientação para corresponder ao seu robô
        //IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                //RevHubOrientationOnRobot.LogoFacingDirection.UP,
                //RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Sem isso, a orientação do REV Hub é considerada logo up / USB forward
        //imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.a) {
                //imu.resetYaw();
            }

            double botHeading = 0;//imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontRightPower = (rotY + rotX + rx) / denominator;
            double backRightPower = (rotY - rotX + rx) / denominator;
            double frontLeftPower = (rotY - rotX - rx) / denominator;
            double backLeftPower = (rotY + rotX - rx) / denominator;

            telemetry.addData("fLeft power",frontLeftPower);
            telemetry.addData("bLeft power",backLeftPower);
            telemetry.addData("BR power",backRightPower);
            telemetry.addData("FR power",frontRightPower);
            telemetry.addData("imu angle",botHeading);
            telemetry.update();
        }    }}