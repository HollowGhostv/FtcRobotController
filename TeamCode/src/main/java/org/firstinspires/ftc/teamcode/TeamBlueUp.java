package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous

public class TeamBlueUp extends OpMode
{ //AutonomoTony Inicio//
    private DcMotor FL; //Motor Mecanum
    private DcMotor BL; //Motor Mecanum
    private DcMotor FR; //Motor Mecanum
    private DcMotor BR; //Motor Mecanum
    private DcMotor Intake1; //Motor Intake IN
    private DcMotor Intake2; //Motor Intake OUT
    private DcMotor Shooter; //Motor Shooter

    boolean Step1 = false;
    boolean Step2 = false;
    boolean ChangeStep2 = false;
    boolean Step3 = false;
    boolean ChangeStep3 = false;
    boolean Step4 = false;
    boolean ChangeStep4 = false;
    boolean Step5 = false;

    private GoBildaPinpointDriver odo; //Odometry
    @Override

    public void init()
    { //public init inicio
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

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

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

    } //public init final

    @Override
    public void loop() {    //inicio void loop

        Pose2D currentPos = odo.getPosition();

        if (currentPos.getX(DistanceUnit.CM) > -164 && !Step1) {
            FL.setPower(-0.4);
            FR.setPower(-0.4);
            BL.setPower(-0.4);
            BR.setPower(-0.4);
            Shooter.setPower(-0.64);
            Intake2.setPower(0);
        } else if (currentPos.getX(DistanceUnit.CM) <= -164 && !Step1) {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            Shooter.setPower(-0.64);
            Intake2.setPower(1);
            Step1 = true;
            ChangeStep2 = true;
        }
        if (odo.getHeading(AngleUnit.DEGREES) > -20 && !Step2 && ChangeStep2) {
            FL.setPower(-0.4);
            FR.setPower(0.4);
            BL.setPower(-0.4);
            BR.setPower(0.4);
        } else if (odo.getHeading(AngleUnit.DEGREES) <= -20 && !Step2 && ChangeStep2) {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            Shooter.setPower(0);
            Intake2.setPower(-0.1);
            Step2 = true;
            odo.resetPosAndIMU();
            Pose2D startPos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
            odo.setPosition(startPos);
            ChangeStep3 = true;
        }
        if (currentPos.getX(DistanceUnit.CM) < 45 && !Step3 && ChangeStep3) {
            FL.setPower(0.4);
            FR.setPower(0.4);
            BL.setPower(0.4);
            BR.setPower(0.4);
        } else if (currentPos.getX(DistanceUnit.CM) >= 45 && !Step3 && ChangeStep3)
        {
            stop();
            Step3 = true;
        }

        odo.update();
        telemetry.addData("X: ", currentPos.getX(DistanceUnit.CM));
        telemetry.addData("Y: ", currentPos.getY(DistanceUnit.CM));
        telemetry.addData("Angle: ", currentPos.getHeading(AngleUnit.DEGREES));
        telemetry.update();

    }    //final void loop
} //AutonomoTony Final//
