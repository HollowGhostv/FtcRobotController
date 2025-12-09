package org.firstinspires.ftc.teamcode.Arcadia;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Map
{
    private DcMotor FL, FR, BL, BR;
    private GoBildaPinpointDriver odo;

    // Ganancias del controlador
    private final double kP_xy = 0.10;      // Control de posición
    private final double kP_heading = 2.0;  // Giro suave pero firme
    private final double minPower = 0.08;   // Para romper fricción
    private final double maxPower = 0.55;   // Limite seguro

    // Anti-rotación por desbalance del robot
    private final double antiRotateGain = 0.15;

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
        Pose2D pos = odo.getPosition();

        double x = pos.getX(DistanceUnit.INCH);
        double y = pos.getY(DistanceUnit.INCH);
        double heading = pos.getHeading(AngleUnit.RADIANS);

        // Errores
        double dx = targetX - x;
        double dy = targetY - y;

        // Distancia al objetivo
        double distance = Math.hypot(dx, dy);

        // Tolerancia
        if (distance < 0.8 && Math.abs(AngleUnit.normalizeDegrees(targetAngle - pos.getHeading(AngleUnit.DEGREES))) < 2)
        {
            stop();
            return true;
        }

        // --- CONTROL PROPORCIONAL EN X Y Y (global) ---
        double powerX_global = dx * kP_xy;
        double powerY_global = dy * kP_xy;

        // Desacelerar suavemente al acercarse
        double slowFactor = Math.min(1.0, distance / 8);
        powerX_global *= slowFactor;
        powerY_global *= slowFactor;

        // --- ROTACIÓN GLOBAL → ROBOT CENTRIC ---
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double powerX_robot = powerX_global * cos - powerY_global * sin;
        double powerY_robot = powerX_global * sin + powerY_global * cos;

        // --- CONTROL PROPORCIONAL DE HEADING ---
        double targetRad = Math.toRadians(targetAngle);
        double headingError = AngleUnit.normalizeRadians(targetRad - heading);
        double turn = headingError * kP_heading;

        // --- ANTI-ROTACION POR STRAFE (COMPENSA PESO) ---
        double antiRotate = antiRotateGain * powerX_robot;

        turn -= antiRotate;

        // --- Aplicar potencias ---
        double fl =  powerY_robot - powerX_robot - turn;
        double fr =  powerY_robot + powerX_robot + turn;
        double bl =  powerY_robot + powerX_robot - turn;
        double br =  powerY_robot - powerX_robot + turn;

        // Normalizar potencias
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        // Aplicar mínimo para vencer fricción
        fl = applyMinPower(fl);
        fr = applyMinPower(fr);
        bl = applyMinPower(bl);
        br = applyMinPower(br);

        // Aplicar máximo de seguridad
        fl *= maxPower;
        fr *= maxPower;
        bl *= maxPower;
        br *= maxPower;

        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);

        return false;
    }

    private double applyMinPower(double p)
    {
        if (Math.abs(p) < minPower && Math.abs(p) > 0)
            return Math.signum(p) * minPower;
        return p;
    }

    private void stop()
    {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }
}
