package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class AutonomoTony extends LinearOpMode
{
    DcMotor FL;
    DcMotor BL;
    DcMotor FR;
    DcMotor BR;
    DcMotor Intake1;
    DcMotor Intake2;
    DcMotor Shooter;


    GoBildaPinpointDriver ooo;
    @Override

    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
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

                sleep(5000);


        }





    }
}
