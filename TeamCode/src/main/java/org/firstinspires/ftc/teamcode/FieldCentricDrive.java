package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp

public class FieldCentricDrive extends OpMode {

    GoBildaPinpointDriver odo;

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private DcMotor Shooter;
    private DcMotor Intake1;
    private DcMotor Intake2;

    @Override

    public void init()
    {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");

        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        Pose2D startPos = new Pose2D(DistanceUnit.MM, -923.925, 1601.14, AngleUnit.RADIANS, 0);
        odo.setPosition(startPos);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

    }

    public void move()
    {
        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pose = odo.getPosition();
        double heading = pose.getHeading(AngleUnit.RADIANS);

        double rot_angle = -heading;

        double cos = Math.cos(rot_angle);
        double sin = Math.sin(rot_angle);

        double globalStrafe = strafe * cos - forward * sin;
        double globalForward = strafe * sin + forward * cos;

        double[] newSpeed = new double[4];

        newSpeed[0] = (globalForward - globalStrafe - rotate) * 0.5;
        newSpeed[1] = (globalForward + globalStrafe + rotate) * 0.5;
        newSpeed[2] = (globalForward + globalStrafe - rotate) * 0.5;
        newSpeed[3] = (globalForward - globalStrafe + rotate) * 0.5;

        double max = Math.abs(newSpeed[0]);
        for (int i = 1; i < newSpeed.length; i++)
        {
            if (Math.abs(newSpeed[i]) > max)
            {
                max = Math.abs(newSpeed[i]);
            }
        }

        if (max > 1.0)
        {
            for (int i = 0; i < newSpeed.length; i++)
            {
                newSpeed[i] /= max;
            }
        }

        FL.setPower(newSpeed[0]);
        FR.setPower(newSpeed[1]);
        BL.setPower(newSpeed[2]);
        BR.setPower(newSpeed[3]);



        telemetry.addData("Robot X Pos: ", pose.getX(DistanceUnit.MM));
        telemetry.addData("Robot Y Pos: ", pose.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("GlobalForward: ", globalForward);
        telemetry.addData("GlobalStrafe: ", globalStrafe);
        telemetry.addData("Rotate: ", rotate);
        telemetry.addData("Speed FL", newSpeed[0]);
        telemetry.addData("Speed FR", newSpeed[1]);
        telemetry.addData("Speed BL", newSpeed[2]);
        telemetry.addData("Speed BR", newSpeed[3]);
    }

    public void loop()
    {
        move();

        if (gamepad1.right_trigger != 0)
        {
            Shooter.setPower(-0.6);
        }
        else if (gamepad1.left_trigger != 0)
        {
            Shooter.setPower(0);
        }

        if (gamepad1.right_bumper)
        {
            Intake2.setPower(1);
        }
        else
        {
            Intake2.setPower(-0.1);
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


        Pose2D pose = odo.getPosition();
        telemetry.addData("Robot X: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Y: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot Angle: ", pose.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        odo.update();
    }
}
