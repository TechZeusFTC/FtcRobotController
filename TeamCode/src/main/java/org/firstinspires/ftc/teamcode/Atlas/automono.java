package org.firstinspires.ftc.teamcode.Atlas;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "Auto imu")
public final class automono extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");

        Servo ServoMA = hardwareMap.servo.get("MA");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        ServoMA.setPosition(0.6);
        while (true) {
            if (!(runtime.seconds() < 0.2)) break;
        }
        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backRightMotor.setPower(0.3);

        runtime.reset();


        //setar power = 0.5

        while (true) {
            if (!(runtime.seconds() < 4.3)) break;
        }

        frontLeftMotor.setPower(-0.3);
        backLeftMotor.setPower(-0.3);
        frontRightMotor.setPower(-0.3);
        backRightMotor.setPower(-0.3);

        while (true) {
            if (!(runtime.seconds() < 6.5)) break;
        }

        frontLeftMotor.setPower(0.3);
        backLeftMotor.setPower(0.3);
        frontRightMotor.setPower(0.3);
        backRightMotor.setPower(0.3);

        while (true) {
            if (!(runtime.seconds() < 8)) break;
        }


        //seta power dos motores pra 0
    }
}
