package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//comp version - final
public class Drivetrain {
    protected DcMotor topRight;
    protected DcMotor bottomRight;
    protected DcMotor topLeft;
    protected DcMotor bottomLeft;
    protected Telemetry telemetry;
    //    private PIDCoefficients pidCoefficientDistance;
//    private PIDCoefficients pidCoefficientTurning;
    protected ElapsedTime timeX;

    protected static final int TICKS_PER_ROTATION = 679 * (4/3);
    protected static final int WHEEL_DIAMETER = 4;
    protected static final double BOT_DIAMETER = 17.5;
    protected static final double BOT_CIRCUMFERENCE = Math.PI*BOT_DIAMETER;
    protected ElapsedTime mRunTime;

//    private double p_distance = 0.05;
//    private double i_distance = 0.0000004;
//    private double d_distance = 0;
//    private double p_turn = -0.0065;
//    private double i_turn = -0.00001;
//    private double d_turn = 0;

    private double strafeCoef = 0.75;
    private long strafedTime = System.currentTimeMillis();
    private double cooldownTime = 250;


    public Drivetrain(DcMotor tl, DcMotor bl, DcMotor tr, DcMotor br, Boolean isAuto, Telemetry t, HardwareMap hardwareMap) {
        this.topLeft = tl;
        this.bottomRight = br;
        this.topRight = tr;
        this.bottomLeft = bl;
        this.telemetry = t;
    }

    //김정은
    public void controls(Gamepad gp) {
        //TODO: Code Mecanum Bullshit
        long timeSinceStrafeChange = System.currentTimeMillis() - strafedTime;

//        if (gp.a && strafeCoef >= 0.1 && timeSinceStrafeChange >= cooldownTime) {
//            strafedTime = System.currentTimeMillis();
//            strafeCoef -= 0.1;
//        }
//        else if(gp.x && strafeCoef <= 0.9 && timeSinceStrafeChange >= cooldownTime) {
//            strafedTime = System.currentTimeMillis();
//            strafeCoef += 0.1;
//        }
//        else if(gp.y && strafeCoef >= 0.01 && timeSinceStrafeChange >= cooldownTime) {
//            strafedTime = System.currentTimeMillis();
//            strafeCoef -= 0.01;
//        }
//        else if(gp.b && strafeCoef <= 0.99 && timeSinceStrafeChange >= cooldownTime) {
//            strafedTime = System.currentTimeMillis();
//            strafeCoef += 0.01;
//        }
        float x = (float)(Math.pow(gp.left_stick_y, 3));
        float y = (float)(Math.pow(-gp.left_stick_x, 3));
        float z = (float)(Math.pow(-gp.right_stick_x, 3));
        if (gp.left_stick_button || gp.right_stick_button || gp.right_trigger >= 0.05) {
            x /=3;
            y /=3;
            z /=3;
        }
        float y_corrected = y;
        if(Math.abs(y) > 0.34) {
            y_corrected = (float)(y * strafeCoef);
        }
        bottomLeft.setPower(((-x)+(y_corrected)+(-z)));
        topLeft.setPower(((-x)+(-y)+(-z)));
        bottomRight.setPower(((x)+(y_corrected)+(-z)));
        topRight.setPower(((x)+(-y)+(-z)));
    }

    public double getStrafeCoef() {
        return strafeCoef;
    }
}
