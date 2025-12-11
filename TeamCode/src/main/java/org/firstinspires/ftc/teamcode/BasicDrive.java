package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BasicDrive extends OpMode
{
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor Intake1;
    private DcMotor Intake2;
    private DcMotor Shooter;

    @Override
    public void init()
    {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        double SPower = -0.65;
        double JoystickY = gamepad1.left_stick_y;
        double JoystickX = -gamepad1.right_stick_x;

        if (gamepad1.dpad_down)
        {
            SPower = -0.65;
        }
        else if (gamepad1.dpad_up)
        {
            SPower = -0.8;
        }

        if (JoystickY != 0)
        {
            FL.setPower(JoystickY);
            FR.setPower(JoystickY);
            BL.setPower(JoystickY);
            BR.setPower(JoystickY);
        }
        else if (JoystickX != 0)
        {
            FL.setPower(JoystickX);
            FR.setPower(-JoystickX);
            BL.setPower(JoystickY);
            BR.setPower(-JoystickX);
        }
        else
        {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }

        if (gamepad1.right_trigger != 0)
        {
            Shooter.setPower(SPower);
        }
        else if (gamepad1.left_trigger != 0)
        {
            Shooter.setPower(0);
        }

        if (gamepad1.a)
        {
            Intake1.setPower(-1);
        }
        else if (gamepad1.b)
        {
            Intake1.setPower(1);
        }
        else
        {
            Intake1.setPower(0);
        }

        if (gamepad1.right_bumper)
        {
            Intake2.setPower(1);
        }
        else
        {
            Intake2.setPower(-0.1);
        }
    }
}
