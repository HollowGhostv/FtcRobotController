package org.firstinspires.ftc.teamcode.Arcadia;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
public class Map
{
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    private GoBildaPinpointDriver odo;

    public Map(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, GoBildaPinpointDriver odometry)
    {
        this.FL = fl;
        this.FR = fr;
        this.BL = bl;
        this.BR = br;
        this.odo = odometry;
    }
    public boolean run_To_Position(double targetX, double targetY, double targetAngle)
    {
        Pose2D Pos = odo.getPosition();
        double PosX = Pos.getX(DistanceUnit.INCH);
        double PosY = Pos.getY(DistanceUnit.INCH);
        double heading = Pos.getHeading(AngleUnit.RADIANS);

        double rot_angle = -heading;
        double sin = Math.sin(rot_angle);
        double cos = Math.cos(rot_angle);

        double dx = targetX - PosX;
        double dy = targetY - PosY;
        double dError = Math.hypot(dx, dy);

        double Tolerancia = 1.0;

        if (dError <= Tolerancia)
        {
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            return true;
        }

        double XPower = dx / dError;
        double YPower = dy / dError;

        double drive  =  YPower * cos - XPower * sin;
        double strafe =  YPower * sin + XPower * cos;


        double targetRad = AngleUnit.DEGREES.toRadians(targetAngle);
        double headingE = AngleUnit.normalizeRadians(targetRad - heading);
        double T = 0.5 * headingE;

        double Y = 0.5 * drive;
        double X = 0.5 * strafe;

        double[] speed = new double[4];

        speed[0] = (Y - X - T);
        speed[1] = (Y + X + T);
        speed[2] = (Y + X - T);
        speed[3] = (Y - X + T);

        FL.setPower(speed[0]);
        FR.setPower(speed[1]);
        BL.setPower(speed[2]);
        BR.setPower(speed[3]);

        return false;
    }
}
