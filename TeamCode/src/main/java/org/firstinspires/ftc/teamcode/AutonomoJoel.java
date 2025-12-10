package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class AutonomoJoel extends LinearOpMode
{
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    DcMotor Intake1;
    DcMotor Intake2;
    DcMotor Shooter;
    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        {
            waitForStart();
            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);
            sleep(4000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(1000);

            FL.setPower(0.5);
            FR.setPower(-0.5);
            BL.setPower(0.5);
            BR.setPower(-0.5);
            sleep(1000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(1000);

            FL.setPower(0.5);
            FR.setPower(0.5);
            BL.setPower(0.5);
            BR.setPower(0.5);
            Intake1.setPower(1);
            sleep(1000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(1000);

            FL.setPower(-0.5);
            FR.setPower(-0.5);
            BL.setPower(-0.5);
            BR.setPower(-0.5);
            Intake1.setPower(1);
            sleep(1000);

            FL.setPower(-0.5);
            FR.setPower(0.5);
            BL.setPower(-0.5);
            BR.setPower(0.5);
            sleep(1000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            Shooter.setPower(0.7);
            sleep(2000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            Shooter.setPower(0.7);
            Intake2.setPower(1);
            sleep(1000);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            Shooter.setPower(0);
            Intake2.setPower(0);
            sleep(2000);
        }
    }
}
