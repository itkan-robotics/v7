package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import java.util.List;

/**
 * Simple diagnostic program to debug Limelight stale frame issues.
 * This program ONLY reads and displays limelight values with no other processing.
 * 
 * CONTROLS:
 * - A: Restart limelight (stop/start)
 * - B: Switch to pipeline 1
 * - X: Switch to pipeline 0
 * - Y: Toggle poll rate (100Hz / 50Hz / 25Hz)
 */
@TeleOp(name="Limelight Diagnostic", group="Test")
public class LimelightDiagnostic extends LinearOpMode {

    private Limelight3A limelight;
    
    // Track frame changes
    private double lastTx = -999;
    private double lastTy = -999;
    private long lastChangeTime = 0;
    private int sameFrameCount = 0;
    private long totalFrames = 0;
    private long staleFrames = 0;
    
    // Track limelight's internal timestamp
    private double lastTimestamp = -1;
    
    // Poll rate cycling
    private int pollRateIndex = 0;
    private int[] pollRates = {100, 50, 25, 10};
    private boolean lastYButton = false;
    private boolean lastAButton = false;

    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        
        // Initialize limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(pollRates[pollRateIndex]);
        limelight.pipelineSwitch(0);
        limelight.start();
        
        telemetry.addData("Status", "Limelight initialized");
        telemetry.addData("Controls", "A=restart, B=pipe1, X=pipe0, Y=pollrate");
        telemetry.addData("Press", "START to begin diagnostic");
        telemetry.update();
        
        waitForStart();
        lastChangeTime = System.currentTimeMillis();
        
        while (opModeIsActive()) {
            long loopStart = System.currentTimeMillis();
            
            // Handle controls
            if (gamepad1.a && !lastAButton) {
                // Restart limelight
                limelight.stop();
                sleep(100);
                limelight.start();
                totalFrames = 0;
                staleFrames = 0;
            }
            lastAButton = gamepad1.a;
            
            if (gamepad1.b) {
                limelight.pipelineSwitch(1);
            }
            if (gamepad1.x) {
                limelight.pipelineSwitch(0);
            }
            
            if (gamepad1.y && !lastYButton) {
                pollRateIndex = (pollRateIndex + 1) % pollRates.length;
                limelight.setPollRateHz(pollRates[pollRateIndex]);
            }
            lastYButton = gamepad1.y;
            
            // Get result directly - no caching, no threading
            LLResult result = limelight.getLatestResult();
            totalFrames++;
            
            if (result == null) {
                telemetry.addData("ERROR", "Result is NULL");
                telemetry.update();
                continue;
            }
            
            // Check validity
            boolean isValid = result.isValid();
            
            // Get basic values
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            
            // Get timing info (if available)
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double timestamp = result.getTimestamp();
            int pipelineIndex = result.getPipelineIndex();
            
            // Check if values actually changed
            boolean txChanged = Math.abs(tx - lastTx) > 0.01;
            boolean tyChanged = Math.abs(ty - lastTy) > 0.01;
            boolean timestampChanged = Math.abs(timestamp - lastTimestamp) > 0.001;
            boolean valuesChanged = txChanged || tyChanged;
            
            if (valuesChanged || timestampChanged) {
                sameFrameCount = 0;
                lastChangeTime = System.currentTimeMillis();
            } else {
                sameFrameCount++;
                if (sameFrameCount > 5) {  // More than 5 identical frames = stale
                    staleFrames++;
                }
            }
            
            lastTx = tx;
            lastTy = ty;
            lastTimestamp = timestamp;
            
            long timeSinceChange = System.currentTimeMillis() - lastChangeTime;
            
            // Get fiducial (AprilTag) info
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            int tagCount = (fiducials != null) ? fiducials.size() : 0;
            String tagIds = "";
            double tagTx = 0, tagTy = 0;
            if (fiducials != null && !fiducials.isEmpty()) {
                StringBuilder sb = new StringBuilder();
                for (LLResultTypes.FiducialResult f : fiducials) {
                    if (sb.length() > 0) sb.append(",");
                    sb.append((int) f.getFiducialId());
                }
                tagIds = sb.toString();
                tagTx = fiducials.get(0).getTargetXDegrees();
                tagTy = fiducials.get(0).getTargetYDegrees();
            }
            
            // Get limelight status
            LLStatus status = limelight.getStatus();
            
            // Display everything
            telemetry.addData("=== LIMELIGHT DIAGNOSTIC ===", "");
            telemetry.addData("Controls", "A=restart B=pipe1 X=pipe0 Y=pollrate");
            telemetry.addLine("");
            
            telemetry.addData("Valid", isValid ? "YES" : "NO");
            telemetry.addData("Pipeline", pipelineIndex);
            telemetry.addData("Poll Rate", "%d Hz", pollRates[pollRateIndex]);
            if (status != null) {
                telemetry.addData("LL Temp", "%.1f C", status.getTemp());
                telemetry.addData("LL FPS", "%.1f", status.getFps());
            }
            telemetry.addLine("");
            
            telemetry.addData("=== RAW VALUES ===", "");
            telemetry.addData("tx", "%.3f %s", tx, txChanged ? "[CHANGED]" : "");
            telemetry.addData("ty", "%.3f %s", ty, tyChanged ? "[CHANGED]" : "");
            telemetry.addData("ta", "%.3f", ta);
            telemetry.addLine("");
            
            telemetry.addData("=== TIMING ===", "");
            telemetry.addData("Timestamp", "%.4f %s", timestamp, timestampChanged ? "[NEW]" : "[SAME]");
            telemetry.addData("Capture Latency", "%.1f ms", captureLatency);
            telemetry.addData("Targeting Latency", "%.1f ms", targetingLatency);
            telemetry.addData("Time Since Change", "%d ms", timeSinceChange);
            telemetry.addLine("");
            
            telemetry.addData("=== APRILTAGS ===", "");
            telemetry.addData("Tags Detected", tagCount);
            telemetry.addData("Tag IDs", tagIds.isEmpty() ? "none" : tagIds);
            if (tagCount > 0) {
                telemetry.addData("Tag tx/ty", "%.2f / %.2f", tagTx, tagTy);
            }
            telemetry.addLine("");
            
            telemetry.addData("=== STALENESS ===", "");
            telemetry.addData("Same Frame Count", sameFrameCount);
            telemetry.addData("Stale Frames", "%d / %d (%.1f%%)", 
                staleFrames, totalFrames, 
                totalFrames > 0 ? (100.0 * staleFrames / totalFrames) : 0);
            telemetry.addData("Status", timeSinceChange > 100 ? ">>> STALE <<<" : "Fresh");
            
            telemetry.update();
        }
        
        limelight.stop();
    }
}

