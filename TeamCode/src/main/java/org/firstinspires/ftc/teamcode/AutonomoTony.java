package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class AutonomoTony extends LinearOpMode
{ //AutonomoTony Inicio//
    DcMotor FL; //Motor Mecanum
    DcMotor BL; //Motor Mecanum
    DcMotor FR; //Motor Mecanum
    DcMotor BR; //Motor Mecanum
    DcMotor Intake1; //Motor Intake IN
    DcMotor Intake2; //Motor Intake OUT
    DcMotor Shooter; //Motor Shooter


    GoBildaPinpointDriver ooo; //Telemetry
    @Override

    public void runOpMode() throws InterruptedException
    { //Public void Inicio//
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
            waitForStart();

        {  //wait for start Inicio//
                FR.setPower(0.3);
                FL.setPower(0.3);
                BR.setPower(0.3);
                BL.setPower(0.3);

                sleep(5000);

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);

                sleep(1000);

                FR.setPower(-0.3);
                FL.setPower(-0.3);
                BR.setPower(0.3);
                BL.setPower(0.3);

                sleep(2500);

                FR.setPower(0.3);
                FL.setPower(0.3);
                BR.setPower(0.3);
                BL.setPower(0.3);
                Intake1.setPower(-1);

                sleep(3000);

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                Intake1.setPower(0);

                sleep(1000);

                FR.setPower(0.3);
                FL.setPower(0.3);
                BR.setPower(-0.3);
                BL.setPower(-0.3);

                sleep(2500);

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                Shooter.setPower(1);

                sleep(5000);

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                Shooter.setPower(1);
                Intake2.setPower(1);

                sleep(1000);

                FR.setPower(0);
                FL.setPower(0);
                BR.setPower(0);
                BL.setPower(0);
                Shooter.setPower(0);
                Intake2.setPower(0);

                sleep(1000);




        } //wait for star final//





    } //Public void Final
} //AutonomoTony Final//
