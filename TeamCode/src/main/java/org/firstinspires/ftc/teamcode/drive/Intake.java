package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    private DcMotor intakeMotor;
    private Servo transfer;

    private long transferTime = System.currentTimeMillis();
    private long posChangeTime = System.currentTimeMillis();
    private int ringsShot = 0;

    double reversePos = 0.41;
    double pushPos = 0.29;

    public Intake(DcMotor intakeMotor, Servo transfer) {
        this.intakeMotor = intakeMotor;
        this.transfer = transfer;
        transfer.setPosition(reversePos);
    }

    public void controls(Gamepad gp) {
        long timeSinceTransfer = System.currentTimeMillis() - transferTime;
        long timeSincePosChange = System.currentTimeMillis() - posChangeTime;
        if(gp.left_bumper) {
            unsucc();
        }
        else if(gp.right_bumper) {
            succ();
        }
        else {
            nosucc();
        }

        if(gp.dpad_left) {
            reverseRingTeleop();
        }
        else if(gp.dpad_right) {
            pushRingTeleop();
        }
        else if(gp.x && timeSinceTransfer >= 500) {
            pushRingCycleTeleop(3);
        }

//        if(gp.dpad_up && timeSincePosChange >= 300) {
//            increaseTransferPos();
//            posChangeTime = System.currentTimeMillis();
//        }
//        else if(gp.dpad_down && timeSincePosChange >= 300) {
//            decreaseTransferPos();
//            posChangeTime = System.currentTimeMillis();
//        }
    }

    public void succ() {
        intakeMotor.setPower(-0.8);
    }

    public void autoSucc(double power) {
        intakeMotor.setPower(-power);
    }

    public void unsucc() {
        intakeMotor.setPower(0.8);
    }

    public void nosucc() {
        intakeMotor.setPower(0.0);
    }


    public void pushRing() {
        transfer.setPosition(pushPos);
        //crServo.setPower(1.0);
    }

    public void pushRingTeleop() {
        transfer.setPosition(pushPos);
    }

    public void reverseRing() {
        transfer.setPosition(reversePos + 0.01);
        //crServo.setPower(-0.5);
    }

    public void reverseRingTeleop() {
        transfer.setPosition(reversePos + 0.01);
    };

    public void increaseTransferPos() {
        pushPos += 0.015;
        reversePos += 0.015;
    }

    public void decreaseTransferPos() {
        pushPos -= 0.015;
        reversePos -= 0.015;
    }

    public double displayTransferPos() {
        return transfer.getPosition();
    }

    public void pushRingCycle(int cycles) {
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis();
        while(cycles > 0) {
            pushRing();
            while (elapsedTime - startTime <= 300) {
                elapsedTime = System.currentTimeMillis();
            }
            startTime = System.currentTimeMillis();
            reverseRing();
            while (elapsedTime - startTime <= 250) {
                elapsedTime = System.currentTimeMillis();
            }
            startTime = System.currentTimeMillis();
            cycles--;
        }
    }

    public void pushRingCycleTeleop(int cycles) {
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis();
        while(cycles > 0) {
            pushRingTeleop();
            while (elapsedTime - startTime <= 200) {
                elapsedTime = System.currentTimeMillis();
            }
            startTime = System.currentTimeMillis();
            reverseRingTeleop();
            while (elapsedTime - startTime <= 200) {
                elapsedTime = System.currentTimeMillis();
            }
            startTime = System.currentTimeMillis();
            cycles--;
        }
    }

    public void stopRing() {
        //crServo.setPower(0.0);
    }
}
