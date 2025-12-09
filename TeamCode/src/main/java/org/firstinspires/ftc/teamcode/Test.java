package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Arcadia.Map;

@Autonomous
public class Test extends OpMode
{
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private GoBildaPinpointDriver odo;

    private DcMotor Intake1;
    private DcMotor Intake2;
    private DcMotor Shooter;
    private Map Map;
    double targetX = 0.0;
    double targetY = 10.0;
    double targetAngle = 0.0;

    @Override
    public void init()
    {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "Odometry");

        Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        Pose2D StartPos = new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0);
        odo.setPosition(StartPos);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        Map = new Map(FL,FR, BL, BR, odo);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
    }

    @Override
    public void loop()
    {
        Pose2D pos = odo.getPosition();
        double PosX = pos.getX(DistanceUnit.INCH);
        double PosY = pos.getY(DistanceUnit.INCH);


        Map.run_To_Position(targetX, targetY, targetAngle);

        telemetry.addData("Target X: ", targetX);
        telemetry.addData("Target Y: ", targetY);
        telemetry.addData("Target Angle: ", targetAngle);
        telemetry.addData("Pos X: ", PosX);
        telemetry.addData("Pos Y: ", PosY);
        telemetry.addData("Angle: ", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("BR: ", BR.getPower());
        odo.update();
        telemetry.update();
    }
}