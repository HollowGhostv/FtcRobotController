package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous
public class TeamRedUp extends OpMode
{
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;
    private DcMotor Intake2;
    private DcMotor Shooter;
    private GoBildaPinpointDriver odo;
    private final  ElapsedTime Time = new ElapsedTime();

    boolean step1 = false;
    boolean TurnStep2 = false;
    boolean step2 = false;
    boolean TurnStep3 = false;
    boolean step3 = false;

    @Override
    public void init()
    {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Odo");

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        Pose2D startPos = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
        odo.setPosition(startPos);

        telemetry.addData("X: ", startPos.getX(DistanceUnit.METER));
        telemetry.addData("Y: ", startPos.getY(DistanceUnit.METER));
        telemetry.addData("Angle: ", startPos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Offset X: ", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Offset Y", odo.getYOffset(DistanceUnit.MM));
    }
    @Override
    public void loop() {
        Pose2D currentPos = odo.getPosition();

        if (currentPos.getX(DistanceUnit.CM) > -170 && !step1)
        {
            FL.setPower(-0.3);
            FR.setPower(-0.3);
            BL.setPower(-0.3);
            BR.setPower(-0.3);
            Shooter.setPower(-0.67);
        }
        else if (currentPos.getX(DistanceUnit.CM) <= -170 && !step1)
        {
            stop();
            Shooter.setPower(-0.67);
            Intake2.setPower(1);
            step1 = true;
            TurnStep2 = true;
        }

        if (currentPos.getHeading(AngleUnit.DEGREES) < -5 && !step2 && TurnStep2)
        {
            Time.startTime();

            if (Time.seconds() > 5)
            {
                FL.setPower(0.3);
                FR.setPower(-0.3);
                BL.setPower(0.3);
                BR.setPower(-0.3);
            }
        }
        else if (currentPos.getHeading(AngleUnit.DEGREES) >= -5 && !step2 && TurnStep2)
        {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            Shooter.setPower(0);
            Intake2.setPower(-0.1);
            step2 = true;
            odo.resetPosAndIMU();
            Pose2D startPos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
            odo.setPosition(startPos);
            TurnStep3 = true;
        }
        if (currentPos.getX(DistanceUnit.CM) < 50 && !step3 && TurnStep3)
        {
            FL.setPower(0.3);
            FR.setPower(0.3);
            BL.setPower(0.3);
            BR.setPower(0.3);
        }
        else if (currentPos.getX(DistanceUnit.CM) >= 50 && !step3 && TurnStep3)
        {
            stop();
            step3 = true;
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
