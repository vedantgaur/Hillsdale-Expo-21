package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Shooter {
    private DcMotor shooter;

    public Shooter(DcMotor shooter) {
        this.shooter = shooter;
    }

    public void controls(Gamepad gp) {
        if(gp.left_bumper) {
            reverseShoot();
        }
        else if(gp.right_bumper) {
            shoot();
        }
        else stopShooter();
    }

    public void shoot() {
        shooter.setPower(-0.68);
    }

    public void shootAuto() {
        shooter.setPower(-0.625);
    }

    public void shootFar(double power) {
        shooter.setPower(-power);
    }

    public void reverseShoot() {
        shooter.setPower(0.1);
    }

    public void stopShooter() {
        shooter.setPower(0.0);
    }
}
