package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Intake Shoot Loop", group = "Autonomous")
public class IntakeShootLoop extends LinearOpMode {

    private Shooter shooter;
    private Limelight limelight;
    
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();
    private ElapsedTime alignTimer = new ElapsedTime();

    // Timing constants
    private static final double INTAKE_TIME = 2.0;    // seconds to intake
    private static final double WAIT_TIME = 2;        // seconds to wait
    private static final double SHOOT_TIME = 0.5;     // seconds to shoot
    private static final double ALIGN_TIMEOUT = 0.5;  // max seconds to wait for alignment

    // Shooter velocity
    private double targetShooterVelocity = Shooter.DEFAULT_TARGET_SHOOTER_VELOCITY;
    private boolean shooterRunning = false;
    private boolean shooting = false;  // Track if waiting for alignment before shooting

    enum State {
        INTAKE,
        WAIT,
        SHOOT
    }
    private State state;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
        
        shooter.setIndexerMiddle();
        shooter.blockShooter();
        shooter.setTurretHome();  // Start turret at home
        
        limelight.start();
        limelight.switchPipeline(0);

        telemetry.addLine("Initialized - Ready to start");
        telemetry.update();

        waitForStart();

        // Start with intake
        state = State.INTAKE;
        timer.reset();
        shooter.runIntakeSystem(Shooter.INTAKE_POWER);
        shooter.setTurretHome();  // Release turret while intaking

        while (opModeIsActive()) {
            // Update shooter motor (bang-bang control)
            shooter.updateShooter(shooterRunning, targetShooterVelocity);

            switch (state) {
                case INTAKE:
                    // Intaking for 2 seconds, turret released
                    if (timer.seconds() >= INTAKE_TIME) {
                        // Stop intake, start waiting/aligning
                        shooter.stopIntakeSystem();
                        shooterRunning = true;  // Start spinning up shooter
                        shooting = true;        // Waiting for alignment
                        alignTimer.reset();
                        state = State.WAIT;
                        timer.reset();
                    }
                    break;

                case WAIT:
                    // Waiting - spin up shooter and align turret
                    limelight.update();
                    limelightTurretAutoAlign();
                    
                    if (timer.seconds() >= WAIT_TIME) {
                        // Move to shooting state
                        state = State.SHOOT;
                        shootTimer.reset();
                        alignTimer.reset();
                        shooting = true;
                    }
                    break;

                case SHOOT:
                    // Shooting - continue aligning and shoot when ready
                    limelight.update();
                    limelightTurretAutoAlign();
                    
                    if (shooting) {
                        // Wait for alignment and shooter speed before shooting
                        boolean isAligned = limelight.isAlignedForShooting();
                        boolean timedOut = alignTimer.seconds() >= ALIGN_TIMEOUT;
                        boolean shooterSpeedReady = Math.abs(shooter.getShooterVelocity() - targetShooterVelocity) <= Shooter.VELOCITY_TOLERANCE;

                        if (shooterSpeedReady && (isAligned || timedOut)) {
                            // Start shooting
                            shooter.unblockShooter();
                            shooter.runIntakeSystem(Shooter.INTAKE_POWER);  // Feed balls to shooter
                            shootTimer.reset();
                            shooting = false;
                            targetShooterVelocity = updateTargetShooterVelocity();
                        }
                    } else {
                        // Already shooting, wait for shoot time
                        if (shootTimer.seconds() >= SHOOT_TIME) {
                            // Stop shooting, go back to intake
                            shooter.blockShooter();
                            shooterRunning = false;
                            shooter.setTurretHome();  // Release turret while intaking
                            shooter.runIntakeSystem(Shooter.INTAKE_POWER);  // Continue intaking
                            state = State.INTAKE;
                            timer.reset();
                        }
                    }
                    break;
            }

            // Telemetry
            telemetry.addData("State", state);
            telemetry.addData("Timer", "%.2f", timer.seconds());
            telemetry.addData("Shooting", shooting);
            telemetry.addData("Shooter Running", shooterRunning);
            telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
            telemetry.addData("Target Velocity", targetShooterVelocity);
            telemetry.addData("Limelight TX", limelight.getTx());
            telemetry.addData("Has Target", limelight.hasTarget());
            telemetry.addData("Turret Angle", shooter.getTurretAngle());
            telemetry.update();
        }
    }

    /**
     * Update target shooter velocity from limelight
     * @return Target velocity in ticks per second
     */
    private double updateTargetShooterVelocity() {
        if (limelight.hasTarget()) {
            return limelight.getTargetShooterTPS();
        } else {
            return Shooter.DEFAULT_TARGET_SHOOTER_VELOCITY;
        }
    }

    /**
     * Auto-align turret using limelight
     * @return true if adjustment was made, false otherwise
     */
    private boolean limelightTurretAutoAlign() {
        if (!limelight.hasTarget()) {
            return false;
        }

        int tagId = limelight.getAprilTagId();
        if (tagId != 24) {
            return false;
        }

        double tx = limelight.getTx();
        double targetOffset = limelight.calculateTargetOffset();
        double error = tx - targetOffset;

        if (Math.abs(error) > Shooter.LIMELIGHT_TOLERANCE) {
            double currentTurretAngle = shooter.getTurretAngle();
            double turretAdjustment = -error * Shooter.LIMELIGHT_KP;
            double newTurretAngle = currentTurretAngle + turretAdjustment;
            shooter.setTurretAngle(newTurretAngle);
            return true;
        }

        return false;
    }
}
