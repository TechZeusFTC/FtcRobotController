package org.firstinspires.ftc.teamcode.Atlas;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class limitador extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Posição do braço quando levantado
        int armUpPosition = 200;

        // Posição do braço quando está abaixado
        int armDownPosition = 0;

        //Encontre um motor no mapa de hardware chamado "MH"
        DcMotor armMotor = hardwareMap.dcMotor.get("MH");

        //Reset o codificador do motor para que ele leia zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define a posição inicial do braço para baixo
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // Se o botão A for pressionado, levante o braço
            if (gamepad2.a) {
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(12);
            }

            // Se o botão B for pressionado, abaixa o braço
            if (gamepad2.b) {
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(-12);
            }

            // Obtém a posição atual do armMotor
            double position = armMotor.getCurrentPosition();

            // Obtém a posição alvo do armMotor
            double desiredPosition = armMotor.getTargetPosition();

            //Mostra a posição do armMotor na telemetria
            telemetry.addData("Encoder Position", position);

            //Mostra a posição alvo do armMotor na telemetria
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
}