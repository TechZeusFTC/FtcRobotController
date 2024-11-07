package org.firstinspires.ftc.teamcode.Atlas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="move", group="Iterative OpMode")
@Config
public class MOVE extends LinearOpMode {

    public static double KP = 0.02;
    public static double KI = 0;
    public static double KD = 0.002;
    public static int ARM_UP_POS = -95;
    public static int ARM_STORED = -10;
    public static int ARM_DOWN_POS = (int) -213;

    public static int ARM_DOWN_MIDDLE = (int) -190;

    public static double SERVO_POS = 0.5;

    public static int ARM_UP_CESTA = -195;




    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();


        PeIDoControler PIDBraco = new PeIDoControler(0, 0, 0);
        // Declara nossos motores
        // Certifique-se de que seus IDs correspondam à sua configuração
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FL");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BL");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FR");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BR");
        DcMotor MH = hardwareMap.dcMotor.get("MH");
        DcMotor BRA = hardwareMap.dcMotor.get("BRA");
        DcMotor Sus1 = hardwareMap.dcMotor.get("Sus1");
        DcMotor BRA2 = hardwareMap.dcMotor.get("BRA2");
        Servo ServoMA = hardwareMap.servo.get("MA");
        // Inverta os motores do lado direito. Isso pode estar errado para sua configuração.
        // Se o seu robô se mover para trás quando for comandado para avançar,
        // inverta o lado esquerdo.
        // Veja a nota sobre isso anteriormente nesta página.
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        MH.setDirection(DcMotorSimple.Direction.REVERSE);
        //BRA.setDirection(DcMotorSimple.Direction.REVERSE);
        //BRA2.setDirection(DcMotorSimple.Direction.REVERSE);
        //Sus2.setDirection(DcMotorSimple.Direction.REVERSE);
        //Sus2.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Recupera o IMU do mapa de hardware
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //Ajuste os parâmetros de orientação para corresponder ao seu robô
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Sem isso, a orientação do REV Hub é considerada logo up / USB forward
        imu.initialize(parameters);



        // Position of the arm when it's lifted
        int armUpPosition = 4000;

        // Position of the arm when it's down
        int armDownPosition = -1;

        int armupPosition = 1500;

        int corerexPoisition = -120;

        int corerexPoisitionzero = 0;

        int alvoArt = 0;

        int alvoArt2 = 0;

        //int Suspen1 = 10;

        //int Suspen2 = 10;


        // Find a motor in the hardware map named "Arm Motor"
        DcMotor armMotor = hardwareMap.dcMotor.get("MH");
        DcMotor MotorArt = hardwareMap.dcMotor.get("BRA");
        DcMotor MotorArte = hardwareMap.dcMotor.get("BRA2");
        //DcMotor SusUp = hardwareMap.dcMotor.get("Sus1");
        //DcMotor SusUp2 = hardwareMap.dcMotor.get("Sus2");


        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorArt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorArte.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //SusUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //SusUp2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Sets the starting position of the arm to the down position
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorArt.setTargetPosition(armDownPosition);
        MotorArt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        MotorArte.setTargetPosition(armDownPosition);
        MotorArte.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //SusUp.setTargetPosition(Suspen1);
        //SusUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //SusUp2.setTargetPosition(Suspen2);
        //SusUp2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ServoMA.setPosition(0.6);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //1 aberta
            //0.6 fechada

            ServoMA.setPosition(+0.4 * gamepad2.right_trigger + 0.6);

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            double kF = 0.08;
            double rx = gamepad1.right_stick_x + kF * x ;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.a) {
                imu.resetYaw();
            }
/*
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontRightPower = (rotY + rotX - rx) / denominator;
            double backRightPower = (rotY - rotX - rx) / denominator;
            double frontLeftPower = (rotY - rotX + rx) / denominator;
            double backLeftPower = (rotY + rotX + rx) / denominator;

             */
            double slower =0.9;
            if (gamepad1.right_bumper){
                slower = 0.25;
            }



            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower*slower);
            backLeftMotor.setPower(backLeftPower*slower);
            frontRightMotor.setPower(frontRightPower*slower);
            backRightMotor.setPower(backRightPower*slower);

            // If the A button is pressed, raise the arm
            if (gamepad2.a) {
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.8);
            }

            // If the B button is pressed, lower the arm
            if (gamepad2.b) {
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(-0.5);
            }

            // If the Y button is pressed, raise the arm
            if (gamepad2.y) {
                armMotor.setTargetPosition(armupPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(1);
            }

            if (gamepad2.dpad_down) {
                alvoArt = ARM_DOWN_POS;
                //int alvoArte = ARM_DOWN_POS;
                MotorArt.setPower(0.0001);
                //MotorArte.setPower(0.02);

            }
            if (gamepad2.dpad_left) {
                alvoArt = ARM_UP_POS;
                //int alvoArte = ARM_UP_POS;
                MotorArt.setPower(0.0001);
                //MotorArte.setPower(0.02);


            }
            if (gamepad2.dpad_up) {
                alvoArt = ARM_STORED;
                MotorArt.setPower(0.0001);// Get the current position of the armMotor
                //int alvoArte = ARM_SORED;
                //MotorArte.setPower(0.02);

            }

            if (gamepad2.right_bumper) {
                alvoArt = ARM_UP_CESTA;
                //int alvoArte = ARM_UP_CESTA;
                MotorArt.setPower(0.0001);
                //MotorArte.setPower(0.02);
            }

            if (gamepad2.dpad_right) {
                alvoArt = ARM_DOWN_MIDDLE;// Get the current position of the armMotor
                //int alvoArte = ARM_DOWN_MIDDLE;
                MotorArt.setPower(0.0001);
                //MotorArte.setPower(0.02);
            }

           /*if (gamepad1.x) {
                SusUp.setTargetPosition(Suspen1);
                SusUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SusUp.setPower(0.5);
            }
    */
           /* double powe = gamepad1.right_trigger;

            Sus1.setPower(powe);

            Sus2.setPower(powe);

            double poweR = gamepad1.left_trigger;

            Sus1.setPower(-poweR);

            Sus2.setPower(-poweR);
                                   */

            double position = armMotor.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = armMotor.getTargetPosition();

            double positionArt = MotorArt.getCurrentPosition();

            // Get the target position of the armMotor
            double DesiredPosition = MotorArt.getTargetPosition();


            PIDBraco.updateConstants(KP, KI, KD);

            double power = PIDBraco.CalculatePID(alvoArt, positionArt);
            MotorArt.setPower(power);


            dashboardTelemetry.addData("armpos", positionArt);
            dashboardTelemetry.addData("armGoal", alvoArt);
            dashboardTelemetry.update();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", alvoArt);

            telemetry.addData("encoder Position", positionArt);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("desired Position", DesiredPosition);

            telemetry.update();


        }
    }
}
