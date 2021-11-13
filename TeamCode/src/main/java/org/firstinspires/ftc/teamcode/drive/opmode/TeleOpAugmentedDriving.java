package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.opmode.TuningController.rpmToTicksPerSecond;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.ArmNoEncoder;
import org.firstinspires.ftc.teamcode.drive.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Shooter;

//import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java class.
 */
@TeleOp(group = "advanced")
public class TeleOpAugmentedDriving extends LinearOpMode {
    // Copy your PIDF Coefficients here
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 10, 11.9);

    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        FIELD_CENTRIC,
        ALIGN_TO_POINT,
        AUTOMATIC_CONTROL,
        AUTOMATIC_2
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(0, 5);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(90);

    // The location we want the bot to automatically go to when we press the B button
    private Pose2d drivePosition = new Pose2d(0, -7, Math.toRadians(0));

    private Pose2d powerShotPosition = new Pose2d(0, 8, Math.toRadians(2));

    double shootTurn1 = Math.toRadians(9);

    double shootTurn2 = Math.toRadians(7);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    private Vector2d targetPosition = new Vector2d(72, -4);

    private long cooldownTime = 300; //300 milliseconds

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize custom cancelable SampleMecanumDrive class
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap.dcMotor.get("intakeMotor"), hardwareMap.servo.get("legsOfDoom"));
        Shooter shooter = new Shooter(hardwareMap.dcMotor.get("shooter"));
        ArmNoEncoder arm = new ArmNoEncoder(hardwareMap.dcMotor.get("armMotor"), hardwareMap.servo.get("yoinker"), hardwareMap.analogInput.get("potentiometer"));

        // We want to turn off velocity control for tel eop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
//        if(PoseStorage.currentPose.getX() == 0.0 && PoseStorage.currentPose.getY() == 0.0 &&
//                PoseStorage.currentPose.getHeading() == 0.0) {
//            drive.setPoseEstimate(new Pose2d(-63, 0, Math.toRadians(0))/*.plus(new Pose2d(0, 0, Math.toRadians(-90)))*/);
//        }
//        else drive.setPoseEstimate(PoseStorage.currentPose);

        headingController.setInputBounds(-Math.PI, Math.PI);

        long grabbedTime = System.currentTimeMillis();
        long positionChangedTime = System.currentTimeMillis();
        long transferChangeTime = System.currentTimeMillis();

        //_________________________________________________________________________________//

        // SETUP MOTOR //
        // Change my id
        DcMotorEx myMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        // Reverse as appropriate
        myMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Turns on bulk reading
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // RUE limits max motor speed to 85% by default
        // Raise that limit to 100%
        MotorConfigurationType motorConfigurationType = myMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        myMotor.setMotorType(motorConfigurationType);

        // Turn on RUN_USING_ENCODER
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set PIDF Coefficients with voltage compensated feedforward value
        myMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                MOTOR_VELO_PID.p, MOTOR_VELO_PID.i, MOTOR_VELO_PID.d,
                MOTOR_VELO_PID.f * 12 / hardwareMap.voltageSensor.iterator().next().getVoltage()
        ));

        // Insert whatever other initialization stuff you do here
        //4000 = power shot, 4175 = auto
        double highGoalVelo = 5250;
        double powerShotVelo = 4000;
        double off = 0;
        double targetVelocity = rpmToTicksPerSecond(highGoalVelo);
        boolean isHighVelo = true;
        boolean isLowVelo = false;

        long time = System.currentTimeMillis();
        long cooldownTime = 500;

        //_______________________________________________________________________//


        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            long timeSinceGrab = System.currentTimeMillis() - grabbedTime;
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            Pose2d driveDirection = new Pose2d();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            long timeSincePosChange = System.currentTimeMillis() - positionChangedTime;
            if(timeSincePosChange >= 400) {
                if(gamepad2.left_stick_button) {
                    drive.setPoseEstimate(poseEstimate.plus(new Pose2d(0, 0, Math.toRadians(-2))));
                    positionChangedTime = System.currentTimeMillis();
                }
                else if(gamepad2.right_stick_button) {
                    drive.setPoseEstimate(poseEstimate.plus(new Pose2d(0, 0, Math.toRadians(2))));
                    positionChangedTime = System.currentTimeMillis();
                }
            }
            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    if(gamepad1.x) {
                        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    }


                    else if(gamepad1.left_trigger >= 0.05 || gamepad1.right_trigger >= 0.05) {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.75 / 3,
                                        -gamepad1.left_stick_x * 0.6 / 3,
                                        -gamepad1.right_stick_x * 0.75 / 3
                                )
                        );
                    }
                    else {
                        drive.setWeightedDrivePower(
                                new Pose2d(
                                        -gamepad1.left_stick_y * 0.75,
                                        -gamepad1.left_stick_x * 0.6,
                                        -gamepad1.right_stick_x * 0.75
                                )
                        );
                    }

//                    if(gamepad1.dpad_down && timeSinceGrab >= cooldownTime) {
//                        grabbedTime = System.currentTimeMillis();
//                        drive.setPoseEstimate(poseEstimate.plus(new Pose2d(0, 0, -poseEstimate.getHeading())));
//                    }

//                    if (gamepad1.b) {
//                        // If the B button is pressed on gamepad1, we generate a lineTo()
//                        // trajectory on the fly and follow it
//                        // We switch the state to AUTOMATIC_CONTROL
//
//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .lineToLinearHeading(drivePosition,
//                                        new MecanumConstraints(new DriveConstraints(
//                                                50, 45, 0.0,
//                                                Math.toRadians(235.0), Math.toRadians(235.0), 0.0), 13.9))
//                                .build();
//
//                        drive.followTrajectoryAsync(traj1);
//
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    }
                     if(gamepad1.y) {
                        drive.turn(shootTurn1);
                        intake.pushRingTeleop();
                        drive.residentSleeper(600);
                        intake.reverseRingTeleop();
                        drive.residentSleeper(600);
                        drive.turn(shootTurn2);
                        intake.pushRingTeleop();
                        drive.residentSleeper(600);
                        intake.reverseRingTeleop();
                    }
//                    else if(gamepad1.a) {
//                        Trajectory traj2 = drive.trajectoryBuilder(poseEstimate)
//                                .lineToLinearHeading(powerShotPosition,
//                                        new MecanumConstraints(new DriveConstraints(
//                                                50, 50, 0.0,
//                                                Math.toRadians(235.0), Math.toRadians(235.0), 0.0), 13.9))
//                                .build();
//
//                        targetVelocity = rpmToTicksPerSecond(powerShotVelo);
//                        isHighVelo = false;
//                        isLowVelo = true;
//                        drive.followTrajectoryAsync(traj2);
//
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    }
                    break;

//                case FIELD_CENTRIC:
//                    // Switch into alignment mode if `a` is pressed
//                    if (gamepad2.a) {
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }
//                    if (gamepad2.y) {
//                        currentMode = Mode.ALIGN_TO_POINT;
//                    }
//
//
//
//                    // Standard teleop control
//                    // Convert gamepad input into desired pose velocity
//                    Vector2d input = new Vector2d(
//                            -gamepad1.left_stick_y,
//                            -gamepad1.left_stick_x
//                    ).rotated(-poseEstimate.getHeading() + Math.toRadians(90));
//
//                    // Pass in the rotated input + right stick value for rotation
//                    // Rotation is not part of the rotated input thus must be passed in separately
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    input.getX(),
//                                    input.getY(),
//                                    -gamepad1.right_stick_x
//                            )
//                    );
//
//                    if (gamepad1.x) {
//                        // If the X button is pressed on gamepad2, we generate a lineTo()
//                        // trajectory on the fly and follow it
//                        // We switch the state to AUTOMATIC_CONTROL
//
//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .lineToLinearHeading(drivePosition)
//                                .build();
//
//                        drive.followTrajectoryAsync(traj1);
//
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    }
//                    break;
//
//                case ALIGN_TO_POINT:
//                    // Switch back into normal driver control mode if `b` is pressed
//                    if (gamepad1.a) {
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }
//
//                    // Create a vector from the gamepad x/y inputs which is the field relative movement
//                    // Then, rotate that vector by the inverse of that heading for field centric control
//                    Vector2d fieldFrameInput = new Vector2d(
//                            -gamepad1.left_stick_y * 0.4,
//                            -gamepad1.left_stick_x * 0.35
//                    );
//                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());
//
//                    // Difference between the target vector and the bot's position
//                    Vector2d difference = targetPosition.minus(poseEstimate.vec());
//                    // Obtain the target angle for feedback and derivative for feedforward
//                    double theta = difference.angle();
//
//                    // Not technically omega because its power. This is the derivative of atan2
//                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());
//
//                    // Set the target heading for the heading controller to our desired angle
//                    headingController.setTargetPosition(theta);
//
//                    // Set desired angular velocity to the heading controller output + angular
//                    // velocity feedforward
//                    double headingInput = (headingController.update(poseEstimate.getHeading())
//                            * DriveConstants.kV + thetaFF)
//                            * DriveConstants.TRACK_WIDTH;
//
//                    // Combine the field centric x/y velocity with our derived angular velocity
//                    driveDirection = new Pose2d(
//                            robotFrameInput,
//                            headingInput
//                    );
//
//                    drive.setWeightedDrivePower(driveDirection);
//
//                    if (gamepad1.left_stick_button) {
//                        // If the B button is pressed on gamepad1, we generate a lineTo()
//                        // trajectory on the fly and follow it
//                        // We switch the state to AUTOMATIC_CONTROL
//
//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .lineToLinearHeading(drivePosition,
//                                        new MecanumConstraints(new DriveConstraints(
//                                                55, 55, 0.0,
//                                                Math.toRadians(235.0), Math.toRadians(235.0), 0.0), 13.9))
//                                .build();
//
//                        drive.followTrajectoryAsync(traj1);
//
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    }
//                    break;

                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.right_stick_button) {
                        drive.cancelFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
            long timeSinceTransferChange = System.currentTimeMillis() - transferChangeTime;
            if(gamepad2.dpad_right && timeSinceTransferChange >= 300) {
                intake.increaseTransferPos();
                transferChangeTime = System.currentTimeMillis();
            }
            else if(gamepad2.dpad_left && timeSinceTransferChange >= 300) {
                intake.decreaseTransferPos();
                transferChangeTime = System.currentTimeMillis();
            }

            intake.controls(gamepad1);
            arm.controls(gamepad2);

            myMotor.setVelocity(targetVelocity);

            double motorVelo = myMotor.getVelocity();
            telemetry.addData("target", targetVelocity);
            telemetry.addData("velocity", motorVelo);
            telemetry.addData("error", targetVelocity - motorVelo);

            telemetry.addData("Transfer", intake.displayTransferPos());
            long timeSinceChange = System.currentTimeMillis() - time;
            if(timeSinceChange >= cooldownTime) {
                if(gamepad2.right_bumper && isHighVelo) {
                    time = System.currentTimeMillis();
                    targetVelocity = rpmToTicksPerSecond(off);
                    isHighVelo = false;
                    isLowVelo = false;
                }
                else if(gamepad2.right_bumper) {
                    time = System.currentTimeMillis();
                    targetVelocity = rpmToTicksPerSecond(highGoalVelo);
                    isHighVelo = true;
                    isLowVelo = false;
                }
                else if(gamepad2.left_bumper && isLowVelo) {
                    time = System.currentTimeMillis();
                    targetVelocity = rpmToTicksPerSecond(off);
                    isHighVelo = false;
                    isLowVelo = false;
                }
                else if(gamepad2.left_bumper) {
                    time = System.currentTimeMillis();
                    targetVelocity = rpmToTicksPerSecond(powerShotVelo);
                    isHighVelo = false;
                    isLowVelo = true;
                }
            }
        }
    }
}
