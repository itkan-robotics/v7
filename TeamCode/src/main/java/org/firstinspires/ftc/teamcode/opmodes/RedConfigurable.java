package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@Autonomous(name = "Red Configurable", group = "Autonomous")
@Configurable // Panels
public class RedConfigurable extends LinearOpMode {
    //ctrl + f new Pose, new Pose(144-
    //ctrl f Math.toRadians(, Math.toRadians(-180-
    //multiply turrent heading by -1
    ElapsedTime stateTimer;
    public static double SHOOT_TIME = 2.0;
    public static double GATE_TIME = 2.0;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    private Drive drive;
    private Limelight limelight;
    private Shooter shooter;

    private int pathState; // Current autonomous path state (state machine)
    int nextPathState;
    private Drive.PathsRed paths; // Paths defined in the Drive subsystem

    // Timer for shooting phases

    // Flag for shooter control - must be updated continuously for bang-bang
    private boolean shooterRunning = false;
    private boolean shooting = false; // Track if shooting sequence has started

    // AprilTag ID detected during init (21, 22, or 23)
    private int detectedTagId = 21;

    // Target shooter velocity (will be updated from limelight)
    private double targetShooterVelocity = Shooter.DEFAULT_TARGET_SHOOTER_VELOCITY;

    // Starting pose - MUST match the beginning of Path1!
    private final Pose2D startPose = new Pose2D(DistanceUnit.INCH, 127.000, 113.500, AngleUnit.RADIANS, Math.toRadians(-90));

    @Override
    public void runOpMode() throws InterruptedException {
        stateTimer = new ElapsedTime();
        // === INIT ===
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        drive = new Drive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        shooter = new Shooter(hardwareMap);

        drive.setStartingPose(startPose);
        paths = new Drive.PathsRed(drive.getFollower());
        pathState = 0;

        limelight.start();
        limelight.switchPipeline(0);
        shooter.blockShooter(); // Start with blocker closed

        telemetry.addLine("INITIALIZED SUCCESSFULLY");
        telemetry.update();
        panelsTelemetry.update(telemetry);

        telemetry.addLine("Waiting");
        telemetry.update();
        // === INIT LOOP - Detect AprilTag ===
        while (!isStarted() && !isStopRequested()) {
            limelight.update();
            shooter.blockShooter();
        }

        // === START ===
        setPathState(0);

        // === MAIN LOOP ===
        while (opModeIsActive()) {
            drive.update(); // Update Pedro Pathing
            limelight.update(); // Update Limelight
            targetShooterVelocity = limelight.getTargetShooterTPS();
            shooter.updateShooter(shooterRunning, targetShooterVelocity);
            autonomousPathUpdate(); // Update autonomous state machine

            panelsTelemetry.update(telemetry);
        }
    }

    /**
     * Sets the path state and starts following the corresponding path
     * @param state the path state to transition to
     */
    public void setPathState(int state) {
        pathState = state;
        switch (pathState) {
            case 0:
                shooterRunning = true;
                shooter.setTurretAngle(-30);
                drive.followPathChain(paths.startToShot, true);
                break;
            case 1:
                stateTimer.reset();
                nextPathState = 2;
                break;
            case 2:
                shooterRunning = false;
                drive.followPathChain(paths.shotToTape2, true);
                shooter.blockShooter();
                break;
            case 3:
                drive.followPathChain(paths.tape2ToShot, true);
                shooter.setIntakeState(Shooter.IntakeState.HOLD);
                nextPathState = 4;
                break;
        }
    }

    /**
     * State machine for autonomous path following.
     * Checks if the current path is complete and transitions to the next state.
     */
    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
            case 3:
                if(!drive.isBusy()) {
                    setPathState(1);
                }
                break;
            case 1:
                if(stateTimer.seconds() > SHOOT_TIME) {
                    setPathState(nextPathState);
                }

                if(shooter.isShooterReady(targetShooterVelocity, limelight.isAligned())) {
                    shooter.unblockShooter();
                    shooter.setIntakeState(Shooter.IntakeState.INTAKING);
                }

                shooter.redlimelightTurretAutoAlign(limelight);
                break;
            case 2:
                if(!drive.isBusy()) {
                    setPathState(3);
                }
        }
    }
}