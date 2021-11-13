package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmNoEncoder {
    public DcMotor armMotor;
    public Servo armServo;
    public AnalogInput potentiometer;
    Telemetry telemetry;

    private int restPosition = -5;
    private int upPosition = -50;  //100 = 30 degrees --> 3.33 = 1 degree? not quite
    private int outPosition = -425;
    private int downPosition = -525;
    private int dropPosition = -640;
    private int hitRingPosition = -750;
    private boolean clawOpen = false;
    private long cooldownTime = 300; //300 milliseconds
    private long grabbedTime = System.currentTimeMillis();
    private long armTime = System.currentTimeMillis();

    private boolean armRest = false;
    private boolean armUp = false;
    private boolean armGrab = true;

    private boolean armRestAuto = false;
    private boolean armUpAuto = true;
    private boolean armOutAuto = false;
    private boolean armOutDownAuto = false;
    private boolean armDropAuto = false;

    public ArmNoEncoder(DcMotor armMotor, Servo armServo, AnalogInput potentiometer) {
        this.armMotor = armMotor;
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(0.0);
        this.armServo = armServo;
        grab();
        this.potentiometer = potentiometer;
    }

    public void controls(Gamepad gp) {
        long timeSinceGrab = System.currentTimeMillis() - grabbedTime;
        long timeSinceArmChange = System.currentTimeMillis() - armTime;

        if(timeSinceArmChange >= cooldownTime) {
            if (gp.a) {
                armTime = System.currentTimeMillis();
                armRun(55, 0.5, 0.25);
                armRest = true;
                armUp = false;
                armGrab = false;
            }
            else if (gp.b) {
                armTime = System.currentTimeMillis();
                armRun(90, 0.5, 0.1);
                armRest = false;
                armUp = true;
                armGrab = false;
            }
            else if (gp.y) {
                armTime = System.currentTimeMillis();
                armRun(175, 0.35, 0.1);
                armRest = false;
                armUp = false;
                armGrab = true;
            }
        }

        if(armRest) {
            armRun(55, 0.4, 0.15);
        }
        else if(armUp) {
            armRun(70, 0.5, 0.1);
        }
        else if(armGrab) {
            armRun(170, 0.35, 0.1);
        }

//        if(gp.right_bumper) {
//            grab();
//        }
//        else if(gp.left_bumper) {
//            release();
//        }

        if(gp.x && timeSinceGrab >= cooldownTime) {
            grabbedTime = System.currentTimeMillis();
            if(clawOpen) {
                grab();
                clawOpen = false;
            }
            else {
                release();
                clawOpen = true;
            }
        }
    }

    public double getVoltage() {
        return potentiometer.getVoltage();
    }

    public void armRun(double angleTo, double maxPower, double p) {
        double deltaAngle = angleTo - getAngle();
        double correctedPower = 0;
        if(deltaAngle >= 0) {
            correctedPower = -0.025 - deltaAngle * 0.01;
        }
        else correctedPower = 0.025 - deltaAngle * 0.01;

        correctedPower += Math.sin(Math.toRadians(getAngle() - 70)) * 0.09;

        if(correctedPower >= maxPower) {
            correctedPower = maxPower;
        }
        else if(correctedPower <= -maxPower) {
            correctedPower = -maxPower;
        }

        armMotor.setPower(correctedPower);
    }

    public void armRest() {
        armRun(20, 0.4, 0.1);
        armRestAuto = true;
        armUpAuto = false;
        armOutAuto = false;
        armOutDownAuto = false;
        armDropAuto = false;

    }

    public void armUp() {
        armRun(60, 0.3, 0.15);
        armRestAuto = false;
        armUpAuto = true;
        armOutAuto = false;
        armOutDownAuto = false;
        armDropAuto = false;
    }

    public void armOut() {
        armRun(125, 0.5, 0.25);
        armRestAuto = false;
        armUpAuto = false;
        armOutAuto = true;
        armOutDownAuto = false;
        armDropAuto = false;
    }

    public void armOutDown() {
        armRun(75, 0.5, 0.2);
        armRestAuto = false;
        armUpAuto = false;
        armOutAuto = false;
        armOutDownAuto = true;
        armDropAuto = false;
    }


    public void armDrop() {
        armRun(170, 0.3, 0.1);
        armRestAuto = false;
        armUpAuto = false;
        armOutAuto = false;
        armOutDownAuto = false;
        armDropAuto = true;
    }

    public double getAngle() {
        double x = potentiometer.getVoltage();
        double sqrtFunction = Math.sqrt(21870000*x*x-24057000*x+19847025);
        return (2700*x + 4455 - sqrtFunction)/(20*x);
    }

    public void update() {
        if(armRestAuto) {
            armRest();
        }
        else if(armUpAuto) {
            armUp();
        }
        else if(armOutAuto) {
            armOut();
        }
        else if(armOutDownAuto) {
            armOutDown();
        }
        else if(armDropAuto) {
            armDrop();
        }
    }


    public void grab() {
        armServo.setPosition(0.14);
    }

    public void release() {
        armServo.setPosition(0.5);
    }
}
