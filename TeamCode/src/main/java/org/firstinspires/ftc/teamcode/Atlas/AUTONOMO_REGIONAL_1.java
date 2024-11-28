package org.firstinspires.ftc.teamcode.Atlas;

import static org.firstinspires.ftc.teamcode.Atlas.MOVE.KD;
import static org.firstinspires.ftc.teamcode.Atlas.MOVE.KI;
import static org.firstinspires.ftc.teamcode.Atlas.MOVE.KP;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Autonomo.MecanumDrive;

@Autonomous(name = "Autonomo 1")
public final class AUTONOMO_REGIONAL_1 extends LinearOpMode {
    public static double alvoArt=0;

    public class elevador implements Action{
        PeIDoControler PIDBraco;
        DcMotor MotorArt;
        public elevador(){
            PIDBraco = new PeIDoControler(KP,KI,KD);
            MotorArt = hardwareMap.dcMotor.get("BRA");
            MotorArt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            MotorArt.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double alvoPessoal = AUTONOMO_REGIONAL_1.alvoArt;
            double positionArt = MotorArt.getCurrentPosition();
            double power = PIDBraco.CalculatePID(alvoPessoal, positionArt);
            MotorArt.setPower(power);
            return true;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Servo ServoMA = hardwareMap.servo.get("MA");
        DcMotor armMotor = hardwareMap.dcMotor.get("MH");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setTargetPosition(-1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Servo ServoPUL = hardwareMap.servo.get("PUL");





        Pose2d initialPose = new Pose2d(-12, 63.5, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Action tab1 = drive.actionBuilder(initialPose)
               /*.afterTime(0,()->{

                             AUTONOMO_REGIONAL_1.alvoArt = -80;//cesta
                            AUTONOMO_REGIONAL_1.alvoArt = -113;//sample
                            AUTONOMO_REGIONAL_1.alvoArt = -10;//guardado
                            AUTONOMO_REGIONAL_1.alvoArt = -45;//Em pé
                            AUTONOMO_REGIONAL_1.alvoArt = 0;

                    // levanta td
                    armMotor.setTargetPosition(1500);
                    armMotor.setTargetPosition(4000);
                    armMotor.setTargetPosition(-1);
                    armMotor.setPower(0.5);
                    ServoMA.setPosition(1);
                    ServoMA.setPosition(0.6);
                    ServoPUL.setPosition(0.6);
                    ServoPUL.setPosition(1);
                        }

*/



                    // AUTONOMO_REGIONAL_1.alvoArt = 0


               .afterTime(0,()->{
                    ServoMA.setPosition(0.495);
                    AUTONOMO_REGIONAL_1.alvoArt = 0;

                })


                .waitSeconds(0.8)
                .afterTime(0, () -> {
                    armMotor.setPower(0.5);
                    armMotor.setTargetPosition(1500);
                    armMotor.setPower(0.5);
                    ServoPUL.setPosition(0.04);

                    //eleva o elevador
                })

                .strafeTo(new Vector2d(-12, 63.5))
                .strafeTo(new Vector2d(0, 49 ))
                .waitSeconds(2)
                .afterTime(0, () -> {
                    AUTONOMO_REGIONAL_1.alvoArt = -92;//

                })
                .waitSeconds(0.8)
                .afterTime(0, () -> {
                    AUTONOMO_REGIONAL_1.alvoArt = -148;//

                    //eleva o elevador
                })
                .waitSeconds(1.5)
                    .afterTime(0, () -> {
                    AUTONOMO_REGIONAL_1.alvoArt = -113;//sample
                    //bate coisa na barra
                })
                .waitSeconds(0.5)
                .afterTime(0, () -> {
                    ServoMA.setPosition(1);
                    //solta servo
                })
                .strafeTo(new Vector2d(0, 54))
                .afterTime(0, () -> {
                    ServoMA.setPosition((0.495));
                    // controlar o que não é da tesoura----------------------------------------------------
                    AUTONOMO_REGIONAL_1.alvoArt = -20;//guardado
                })
                .waitSeconds(2)
                .afterTime(0, () -> {
                    AUTONOMO_REGIONAL_1.alvoArt = 0;//guardado
                    //---------------------------------------------------
                    armMotor.setPower(0.3);
                    armMotor.setTargetPosition(-1);
                    armMotor.setPower(0.3);
                    //abaixa e guarda garra
                })

                .strafeTo(new Vector2d(-36, 36))
                .strafeTo(new Vector2d(-36, 8))
                .strafeTo(new Vector2d(-44, 8))
                .strafeTo(new Vector2d(-44,59))
                .strafeTo(new Vector2d(-44,8))
                .strafeTo(new Vector2d(-54,8))
                .strafeTo(new Vector2d(-54,65))
                .afterTime(0, () -> {
                    ServoMA.setPosition(1);
                    ServoPUL.setPosition(0.6);

                })
                //.strafeTo(new Vector2d(-50,8))
                // .strafeTo(new Vector2d(-60,12))
                .build();


        waitForStart();
        Actions.runBlocking(
                new ParallelAction(
                        tab1,
                        new elevador()

                )

        );
    }
}


