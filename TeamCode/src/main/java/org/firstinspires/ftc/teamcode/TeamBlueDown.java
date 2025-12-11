package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class TeamBlueDown extends OpMode
{
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private GoBildaPinpointDriver odo;
    private DcMotor Intake2;
    private DcMotor Shooter;
    private final ElapsedTime Time = new ElapsedTime();
    boolean step1 = false;
    boolean TurnStep2 = false;
    boolean step2 = false;
    boolean TurnStep3 = false;
    boolean step3 = false;
    boolean TurnStep4 = false;
    boolean step4 = false;
    boolean TurnStep5 = false;
    boolean step5 = false;
    boolean TurnStep6 = false;
    boolean step6 = false;
    boolean TurnStep7 = false;
    boolean step7 = false;
    @Override
    public void init()
    {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Odo");

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        Pose2D startPos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        odo.setPosition(startPos);

        telemetry.addData("X: ", startPos.getX(DistanceUnit.METER));
        telemetry.addData("Y: ", startPos.getY(DistanceUnit.METER));
        telemetry.addData("Angle: ", startPos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Offset X: ", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Offset Y", odo.getYOffset(DistanceUnit.MM));
    }

    @Override
    public void loop()
    {
        Pose2D currentPos = odo.getPosition();

        if (currentPos.getX(DistanceUnit.CM) < 35 && !step1)
        {
            FL.setPower(0.3);
            FR.setPower(0.3);
            BL.setPower(0.3);
            BR.setPower(0.3);
            Shooter.setPower(-0.7);
        }
        else if (currentPos.getX(DistanceUnit.CM) >= 35 && !step1)
        {
            stop();
            step1 = true;
            TurnStep2 = true;
        }
        if (odo.getHeading(AngleUnit.DEGREES) > -90 && !step2 && TurnStep2)
        {
            FL.setPower(0.3);
            FR.setPower(-0.3);
            BL.setPower(0.3);
            BR.setPower(-0.3);
        }
        else if (odo.getHeading(AngleUnit.DEGREES) <= -90 && !step2 && TurnStep2)
        {
            stop();
            step2 = true;
            odo.resetPosAndIMU();
            Pose2D startPos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
            odo.setPosition(startPos);
            TurnStep3 = true;
        }

        if (currentPos.getX(DistanceUnit.CM) < 45 && !step3 && TurnStep3)
        {
            FL.setPower(0.3);
            FR.setPower(0.3);
            BL.setPower(0.3);
            BR.setPower(0.3);
        }
        else if (currentPos.getX(DistanceUnit.CM) >= 45 && !step3 && TurnStep3)
        {
            stop();
            step3 = true;
            TurnStep4 = true;
        }

        if (currentPos.getHeading(AngleUnit.DEGREES) < 175 && !step4 && TurnStep4)
        {
            FL.setPower(0.3);
            FR.setPower(-0.3);
            BL.setPower(0.3);
            BR.setPower(-0.3);
        }
        else if (currentPos.getHeading(AngleUnit.DEGREES) >= 175 && !step4 && TurnStep4)
        {
            stop();
            step4 = true;
            Intake2.setPower(1);
            TurnStep5 = true;
        }

        if (currentPos.getHeading(AngleUnit.DEGREES) > 110 && !step5 && TurnStep5)
        {
            Time.startTime();

            if (Time.seconds() < 5)
            {
                FL.setPower(-0.3);
                FR.setPower(0.3);
                BL.setPower(-0.3);
                BR.setPower(0.3);

                Shooter.setPower(0);
            }
        }
        else if (currentPos.getHeading(AngleUnit.DEGREES) <= 110 && !step5 && TurnStep5)
        {
            stop();
            odo.resetPosAndIMU();
            Pose2D startPos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
            odo.setPosition(startPos);
            step5 = true;
            TurnStep6 = true;
        }

        if (currentPos.getX(DistanceUnit.CM) < 35 && !step6 && TurnStep6)
        {
            FL.setPower(0.3);
            FR.setPower(0.3);
            BL.setPower(0.3);
            BR.setPower(0.3);
        }
        else if (currentPos.getX(DistanceUnit.CM) >= 35 && !step6 && TurnStep6)
        {
            stop();
            step6 = true;
            TurnStep7 = true;
        }

        if (currentPos.getHeading(AngleUnit.DEGREES) < 90 && !step7 && TurnStep7)
        {
            FL.setPower(-0.3);
            FR.setPower(0.3);
            BL.setPower(-0.3);
            BR.setPower(0.3);
        }
        else if (currentPos.getHeading(AngleUnit.DEGREES) >= 90 && !step7 && TurnStep7)
        {
            stop();
            Shooter.setPower(0);
            Intake2.setPower(0);
            step7 = true;
        }

        odo.update();
        telemetry.addData("X: ", currentPos.getX(DistanceUnit.CM));
        telemetry.addData("Y: ", currentPos.getY(DistanceUnit.CM));
        telemetry.addData("Angle: ", currentPos.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    public void stop()
    {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
