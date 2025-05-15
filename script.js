document.addEventListener('DOMContentLoaded', () => {
    const canvas = document.getElementById('simulationCanvas');
    const ctx = canvas.getContext('2d');

    // --- DOM Elements for UI ---
    // ... (Same as previous script.js)
    const timeStepInput = document.getElementById('timeStep');
    const pixelsPerMeterInput = document.getElementById('pixelsPerMeter');
    const maxRobotSpeedMPSInput = document.getElementById('maxRobotSpeedMPS');
    const motorResponseFactorInput = document.getElementById('motorResponseFactor');
    const sensorNoiseProbInput = document.getElementById('sensorNoiseProb');        
    const movementPerturbFactorInput = document.getElementById('movementPerturbFactor'); 
    const motorDeadbandPWMInput = document.getElementById('motorDeadbandPWM');       
    
    const robotActualWidthInput = document.getElementById('robotWidthInput_actual');
    const robotActualLengthInput = document.getElementById('robotLengthInput_actual');
    const sideSensorSpreadInput = document.getElementById('sideSensorSpreadInput');
    const sensorForwardOffsetInput = document.getElementById('sensorForwardOffsetInput');

    const arduinoKpInput = document.getElementById('arduino_kp');
    const arduinoKiInput = document.getElementById('arduino_ki');
    const arduinoKdInput = document.getElementById('arduino_kd');
    const arduinoVelBaseInput = document.getElementById('arduino_velBase');
    const arduinoVelRecInput = document.getElementById('arduino_velRec');
    const arduinoVelRevRecInput = document.getElementById('arduino_velRevRec');
    const arduinoIntegralMaxInput = document.getElementById('arduino_integralMax');

    const startButton = document.getElementById('startButton');
    const stopButton = document.getElementById('stopButton');
    const resetButton = document.getElementById('resetButton');

    const errorValSpan = document.getElementById('errorVal');
    const pValSpan = document.getElementById('pVal');
    const iValSpan = document.getElementById('iVal');
    const dValSpan = document.getElementById('dVal');
    const arduinoAjusteValSpan = document.getElementById('arduinoAjusteVal');
    const vLeftValSpan = document.getElementById('vLeftVal');
    const vRightValSpan = document.getElementById('vRightVal');

    const editorStatusSpan = document.getElementById('editorStatus');
    const editTrackButton = document.getElementById('editTrackButton');
    const clearTrackButton = document.getElementById('clearTrackButton');
    const finishEditingButton = document.getElementById('finishEditingButton');


    // --- Simulation Parameters ---
    // ... (Same as previous script.js)
    let simTimeStep, pixelsPerMeter, maxPhysicalSpeed_mps, currentMotorResponseFactor;
    let sensorNoiseProbability, movementPerturbationFactor, motorDeadbandPWMValue; 
    let currentRobotWheelbase_m, currentRobotLength_m, sensorSideSpread_m, sensorForwardProtrusion_m;
    
    let arduinoKp, arduinoKi, arduinoKd;
    let arduinoVelBase, arduinoVelRec, arduinoVelRevRec, arduinoIntegralMax;

    let arduinoErrorPID = 0, arduinoErrorPrevioPID = 0, arduinoTerminoProporcional = 0;
    let arduinoTerminoIntegral = 0, arduinoTerminoDerivativo = 0, arduinoAjustePID = 0;
    let ultimaPosicionConocidaLinea = 0;

    let currentApplied_vR_mps = 0;
    let currentApplied_vL_mps = 0;

    const wheelWidth_px = 8; 
    const wheelRadius_px = 12; 
    const sensorRadiusPixels = 4;
    const lineWidthPixels = 10;
    const trackPointRadiusPixels = 5;

    let robot = { x_m: 0, y_m: 0, angle_rad: 0, trail: [] };


    // UPDATED Default trackPoints in pixels (approximating the new image)
    let trackPoints_pixels = [
        { x: 100, y: 100 }, // Start
        { x: 300, y: 100 },
        { x: 350, y: 150 },
        { x: 350, y: 250 },
        { x: 400, y: 300 },
        { x: 500, y: 300 },
        { x: 550, y: 250 },
        { x: 550, y: 150 },
        { x: 600, y: 100 },
        { x: 700, y: 100 },
        { x: 750, y: 150 },
        { x: 750, y: 250 },
        { x: 700, y: 300 },
        { x: 650, y: 350 }, // Start S-curve
        { x: 650, y: 450 },
        { x: 700, y: 500 },
        { x: 600, y: 500 },
        { x: 550, y: 450 }, // End S-curve
        { x: 550, y: 350 },
        { x: 500, y: 300 }, // Connecting back (already listed, creating sharpness)
        { x: 400, y: 300 }, // Connecting back (already listed)
        { x: 350, y: 250 }, // Connecting back
        { x: 300, y: 280 }, // Slight inward curve
        { x: 200, y: 350 },
        { x: 150, y: 450 },
        { x: 200, y: 500 },
        { x: 300, y: 500 },
        { x: 350, y: 450 },
        { x: 300, y: 400 },
        { x: 150, y: 300 },
        { x: 100, y: 200 },
        { x: 100, y: 100 }  // Close loop
    ];


    let simulationRunning = false;
    // ... (Rest of the script.js is IDENTICAL to the previous complete version)
    let animationFrameId;
    let accumulator = 0;
    let lastFrameTime = performance.now();
    let isEditingTrack = false;

    function degreesToRadians(degrees) { return degrees * (Math.PI / 180); }
    function radiansToDegrees(radians) { return radians * (180 / Math.PI); }
    function clamp(value, min, max) { return Math.min(Math.max(value, min), max); }
    function getMousePos(canvasDom, event) {
        const rect = canvasDom.getBoundingClientRect();
        return { x: event.clientX - rect.left, y: event.clientY - rect.top };
    }

    function drawRobot() {
        ctx.save();
        ctx.translate(robot.x_m * pixelsPerMeter, robot.y_m * pixelsPerMeter);
        ctx.rotate(robot.angle_rad);
        const robotWheelbasePx = currentRobotWheelbase_m * pixelsPerMeter;
        const robotLengthPx = currentRobotLength_m * pixelsPerMeter;
        ctx.fillStyle = '#555555'; 
        const visualWheelDiameter = wheelRadius_px * 2; 
        const visualWheelThickness = wheelWidth_px;    
        ctx.fillRect(-visualWheelDiameter / 2, (-robotWheelbasePx / 2) - (visualWheelThickness / 2), visualWheelDiameter, visualWheelThickness);
        ctx.fillRect(-visualWheelDiameter / 2, (robotWheelbasePx / 2) - (visualWheelThickness / 2), visualWheelDiameter, visualWheelThickness);
        ctx.fillStyle = 'blue';
        ctx.fillRect(-robotLengthPx / 2, -robotWheelbasePx / 2, robotLengthPx, robotWheelbasePx);
        ctx.fillStyle = 'lightblue';
        ctx.beginPath();
        const indicatorTipX = robotLengthPx / 2 + 3; 
        const indicatorBaseX = robotLengthPx / 2 - Math.min(8, robotLengthPx * 0.15); 
        const indicatorBaseSpread = robotWheelbasePx / 3; 
        ctx.moveTo(indicatorTipX, 0); 
        ctx.lineTo(indicatorBaseX, -indicatorBaseSpread / 2); 
        ctx.lineTo(indicatorBaseX, indicatorBaseSpread / 2);  
        ctx.closePath(); ctx.fill();
        ctx.restore();
        if (robot.trail.length > 1) {
            ctx.beginPath(); ctx.strokeStyle = 'rgba(0, 0, 255, 0.3)'; ctx.lineWidth = 2;
            ctx.moveTo(robot.trail[0].x_px, robot.trail[0].y_px);
            for (let i = 1; i < robot.trail.length; i++) { ctx.lineTo(robot.trail[i].x_px, robot.trail[i].y_px); }
            ctx.stroke();
        }
    }

    function getSensorPositions_pixels() {
        const sensorLineCenterX_m = robot.x_m + sensorForwardProtrusion_m * Math.cos(robot.angle_rad);
        const sensorLineCenterY_m = robot.y_m + sensorForwardProtrusion_m * Math.sin(robot.angle_rad);
        const perpendicularAngle = robot.angle_rad - Math.PI / 2;
        const lX_m = sensorLineCenterX_m + sensorSideSpread_m * Math.cos(perpendicularAngle);
        const lY_m = sensorLineCenterY_m + sensorSideSpread_m * Math.sin(perpendicularAngle);
        const cX_m = sensorLineCenterX_m; const cY_m = sensorLineCenterY_m;
        const rX_m = sensorLineCenterX_m - sensorSideSpread_m * Math.cos(perpendicularAngle);
        const rY_m = sensorLineCenterY_m - sensorSideSpread_m * Math.sin(perpendicularAngle);
        return {
            left:   { x: lX_m * pixelsPerMeter, y: lY_m * pixelsPerMeter },
            center: { x: cX_m * pixelsPerMeter, y: cY_m * pixelsPerMeter },
            right:  { x: rX_m * pixelsPerMeter, y: rY_m * pixelsPerMeter }
        };
    }

    function drawSensors(sensorStates) {
        const positions_px = getSensorPositions_pixels();
        drawSensor(positions_px.left, sensorStates.left);
        drawSensor(positions_px.center, sensorStates.center);
        drawSensor(positions_px.right, sensorStates.right);
    }

    function drawSensor(pos_px, isOnLine) {
        ctx.beginPath(); ctx.arc(pos_px.x, pos_px.y, sensorRadiusPixels, 0, 2 * Math.PI);
        ctx.fillStyle = isOnLine ? 'red' : 'gray'; ctx.fill();
        ctx.strokeStyle = 'black'; ctx.lineWidth = 1; ctx.stroke();
    }

    function drawTrack() {
        if (trackPoints_pixels.length < 2) {
            ctx.fillStyle = 'rgba(0,0,0,0.5)';
            trackPoints_pixels.forEach(p => { ctx.beginPath(); ctx.arc(p.x, p.y, trackPointRadiusPixels, 0, Math.PI * 2); ctx.fill(); });
            return;
        }
        ctx.beginPath(); ctx.strokeStyle = 'black'; ctx.lineWidth = lineWidthPixels;
        ctx.moveTo(trackPoints_pixels[0].x, trackPoints_pixels[0].y);
        for (let i = 1; i < trackPoints_pixels.length; i++) { ctx.lineTo(trackPoints_pixels[i].x, trackPoints_pixels[i].y); }
        ctx.stroke();
        if (isEditingTrack || trackPoints_pixels.length > 0) {
            ctx.fillStyle = 'rgba(0,0,0,0.5)';
            trackPoints_pixels.forEach(p => { ctx.beginPath(); ctx.arc(p.x, p.y, trackPointRadiusPixels, 0, Math.PI * 2); ctx.fill(); });
        }
    }

    function isPointOnLine(point_px) {
        if (trackPoints_pixels.length < 2) return false;
        for (let i = 0; i < trackPoints_pixels.length - 1; i++) {
            const p1_px = trackPoints_pixels[i]; const p2_px = trackPoints_pixels[i+1];
            const distSq = distToSegmentSquared(point_px, p1_px, p2_px);
            if (distSq <= (lineWidthPixels / 2 + sensorRadiusPixels) ** 2 ) return true;
        }
        return false;
    }
    
    function distToSegmentSquared(p, v, w) {
        const l2 = (v.x - w.x)**2 + (v.y - w.y)**2;
        if (l2 === 0) return (p.x - v.x)**2 + (p.y - v.y)**2;
        let t = ((p.x - v.x) * (w.x - v.x) + (p.y - v.y) * (w.y - v.y)) / l2;
        t = Math.max(0, Math.min(1, t));
        const projX = v.x + t * (w.x - v.x); const projY = v.y + t * (w.y - v.y);
        return (p.x - projX)**2 + (p.y - projY)**2;
    }

    function fixedUpdate(dt_s) {
        const sensorPositions_px = getSensorPositions_pixels();
        let s_der_active = isPointOnLine(sensorPositions_px.right);
        let s_cen_active = isPointOnLine(sensorPositions_px.center);
        let s_izq_active = isPointOnLine(sensorPositions_px.left);

        if (simulationRunning && sensorNoiseProbability > 0) {
            if (Math.random() < sensorNoiseProbability) s_der_active = !s_der_active;
            if (Math.random() < sensorNoiseProbability) s_cen_active = !s_cen_active;
            if (Math.random() < sensorNoiseProbability) s_izq_active = !s_izq_active;
        }

        const S_DER = s_der_active ? 1 : 0; const S_CEN = s_cen_active ? 1 : 0; const S_IZQ = s_izq_active ? 1 : 0;
        let lineaPerdida = false;

        if (S_DER === 0 && S_CEN === 0 && S_IZQ === 0) { lineaPerdida = true; }
        else if (S_DER === 0 && S_CEN === 1 && S_IZQ === 0) { arduinoErrorPID = 0.0; ultimaPosicionConocidaLinea = 0; }
        else if (S_DER === 0 && S_CEN === 1 && S_IZQ === 1) { arduinoErrorPID = -0.5; ultimaPosicionConocidaLinea = 1; }
        else if (S_DER === 0 && S_CEN === 0 && S_IZQ === 1) { arduinoErrorPID = -2.0; ultimaPosicionConocidaLinea = 1; }
        else if (S_DER === 1 && S_CEN === 1 && S_IZQ === 0) { arduinoErrorPID = 0.5; ultimaPosicionConocidaLinea = 2; }
        else if (S_DER === 1 && S_CEN === 0 && S_IZQ === 0) { arduinoErrorPID = 2.0; ultimaPosicionConocidaLinea = 2; }
        else if (S_DER === 1 && S_CEN === 1 && S_IZQ === 1) { arduinoErrorPID = 0.0; ultimaPosicionConocidaLinea = 0; }
        else if (S_DER === 1 && S_CEN === 0 && S_IZQ === 1) { arduinoErrorPID = arduinoErrorPrevioPID; }
        else { lineaPerdida = true; }
        
        errorValSpan.textContent = arduinoErrorPID.toFixed(2);

        let velRawMotorA_target = 0; let velRawMotorB_target = 0;
        let dirA_adelante = true; let dirB_adelante = true;

        if (lineaPerdida) {
            arduinoTerminoIntegral = 0; arduinoErrorPrevioPID = 0; arduinoAjustePID = 0;
            pValSpan.textContent = "REC"; iValSpan.textContent = "REC"; dValSpan.textContent = "REC";
            if (ultimaPosicionConocidaLinea === 1) {
                velRawMotorA_target = arduinoVelRec; dirA_adelante = true;
                velRawMotorB_target = arduinoVelRevRec; dirB_adelante = false;
            } else if (ultimaPosicionConocidaLinea === 2) {
                velRawMotorA_target = arduinoVelRevRec; dirA_adelante = false;
                velRawMotorB_target = arduinoVelRec; dirB_adelante = true;
            } else {
                velRawMotorA_target = arduinoVelRec; dirA_adelante = true;
                velRawMotorB_target = arduinoVelRec; dirB_adelante = true;
            }
        } else {
            arduinoTerminoProporcional = arduinoKp * arduinoErrorPID;
            arduinoTerminoIntegral += arduinoKi * arduinoErrorPID * dt_s;
            arduinoTerminoIntegral = clamp(arduinoTerminoIntegral, -arduinoIntegralMax, arduinoIntegralMax);
            if (dt_s > 0.0001) { arduinoTerminoDerivativo = arduinoKd * (arduinoErrorPID - arduinoErrorPrevioPID) / dt_s; }
            else { arduinoTerminoDerivativo = 0; }
            arduinoErrorPrevioPID = arduinoErrorPID;
            arduinoAjustePID = arduinoTerminoProporcional + arduinoTerminoIntegral + arduinoTerminoDerivativo;
            velRawMotorA_target = arduinoVelBase - arduinoAjustePID;
            velRawMotorB_target = arduinoVelBase + arduinoAjustePID;
            pValSpan.textContent = arduinoTerminoProporcional.toFixed(2);
            iValSpan.textContent = arduinoTerminoIntegral.toFixed(2);
            dValSpan.textContent = arduinoTerminoDerivativo.toFixed(2);
        }
        arduinoAjusteValSpan.textContent = arduinoAjustePID.toFixed(2);

        let finalVelRawMotorA = velRawMotorA_target;
        let finalVelRawMotorB = velRawMotorB_target;
        if (Math.abs(velRawMotorA_target) < motorDeadbandPWMValue) finalVelRawMotorA = 0;
        if (Math.abs(velRawMotorB_target) < motorDeadbandPWMValue) finalVelRawMotorB = 0;
        
        finalVelRawMotorA = clamp(finalVelRawMotorA, 0, 255);
        finalVelRawMotorB = clamp(finalVelRawMotorB, 0, 255);
        vLeftValSpan.textContent = Math.round(finalVelRawMotorB);
        vRightValSpan.textContent = Math.round(finalVelRawMotorA);

        let target_vR_mps = (dirA_adelante ? 1 : -1) * (finalVelRawMotorA / 255.0) * maxPhysicalSpeed_mps;
        let target_vL_mps = (dirB_adelante ? 1 : -1) * (finalVelRawMotorB / 255.0) * maxPhysicalSpeed_mps;

        currentApplied_vR_mps += (target_vR_mps - currentApplied_vR_mps) * currentMotorResponseFactor;
        currentApplied_vL_mps += (target_vL_mps - currentApplied_vL_mps) * currentMotorResponseFactor;
        
        const L_m = currentRobotWheelbase_m;
        let d_theta_rad = 0;
        if (L_m > 0.001) {
            let standard_d_theta = (currentApplied_vR_mps - currentApplied_vL_mps) / L_m * dt_s; 
            d_theta_rad = -standard_d_theta; 
        }
        let linear_displacement_m = (currentApplied_vR_mps + currentApplied_vL_mps) / 2.0 * dt_s;
        
        if (simulationRunning && movementPerturbationFactor > 0) {
            linear_displacement_m *= (1 + (Math.random() - 0.5) * 2 * movementPerturbationFactor);
            d_theta_rad *= (1 + (Math.random() - 0.5) * 2 * movementPerturbationFactor);
        }
        
        robot.angle_rad += d_theta_rad;
        robot.angle_rad = Math.atan2(Math.sin(robot.angle_rad), Math.cos(robot.angle_rad)); 
        robot.x_m += linear_displacement_m * Math.cos(robot.angle_rad);
        robot.y_m += linear_displacement_m * Math.sin(robot.angle_rad);

        const currentPosPx = { x_px: robot.x_m * pixelsPerMeter, y_px: robot.y_m * pixelsPerMeter };
        if (!robot.trail.length || (currentPosPx.x_px !== robot.trail[robot.trail.length-1].x_px || currentPosPx.y_px !== robot.trail[robot.trail.length-1].y_px) ) {
            robot.trail.push(currentPosPx);
            if (robot.trail.length > 500) robot.trail.shift();
        }
        const boundaryMargin_m = Math.max(currentRobotWheelbase_m, currentRobotLength_m) / 2;
        if (robot.x_m < -boundaryMargin_m || robot.x_m * pixelsPerMeter > canvas.width + boundaryMargin_m * pixelsPerMeter || 
            robot.y_m < -boundaryMargin_m || robot.y_m * pixelsPerMeter > canvas.height + boundaryMargin_m * pixelsPerMeter) {
            stopSimulation();
        }
        return { left: s_izq_active, center: s_cen_active, right: s_der_active };
    }

    function render(sensorStates) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        drawTrack();
        if (simulationRunning || !isEditingTrack) {
            drawRobot();
            if (simulationRunning && sensorStates) drawSensors(sensorStates);
        }
    }

    function gameLoop(currentTime) {
        if (!simulationRunning) return;
        animationFrameId = requestAnimationFrame(gameLoop);
        const frameTime = (currentTime - lastFrameTime) / 1000.0;
        lastFrameTime = currentTime;
        accumulator += frameTime;
        let sensorStatesForRender;
        while (accumulator >= simTimeStep) {
            sensorStatesForRender = fixedUpdate(simTimeStep);
            accumulator -= simTimeStep;
        }
        if (sensorStatesForRender !== undefined || !simulationRunning) {
             render(sensorStatesForRender);
        }
    }

    function loadParameters() {
        simTimeStep = parseFloat(timeStepInput.value);
        pixelsPerMeter = parseFloat(pixelsPerMeterInput.value);
        maxPhysicalSpeed_mps = parseFloat(maxRobotSpeedMPSInput.value);
        currentMotorResponseFactor = parseFloat(motorResponseFactorInput.value);
        currentMotorResponseFactor = clamp(currentMotorResponseFactor, 0.01, 1.0);
        sensorNoiseProbability = parseFloat(sensorNoiseProbInput.value);
        sensorNoiseProbability = clamp(sensorNoiseProbability, 0, 1);
        movementPerturbationFactor = parseFloat(movementPerturbFactorInput.value);
        movementPerturbationFactor = clamp(movementPerturbationFactor, 0, 0.5);
        motorDeadbandPWMValue = parseFloat(motorDeadbandPWMInput.value);
        motorDeadbandPWMValue = clamp(motorDeadbandPWMValue, 0, 50);

        currentRobotWheelbase_m = parseFloat(robotActualWidthInput.value);
        currentRobotLength_m = parseFloat(robotActualLengthInput.value);
        sensorSideSpread_m = parseFloat(sideSensorSpreadInput.value);
        sensorForwardProtrusion_m = parseFloat(sensorForwardOffsetInput.value);
        arduinoKp = parseFloat(arduinoKpInput.value);
        arduinoKi = parseFloat(arduinoKiInput.value);
        arduinoKd = parseFloat(arduinoKdInput.value);
        arduinoVelBase = parseFloat(arduinoVelBaseInput.value);
        arduinoVelRec = parseFloat(arduinoVelRecInput.value);
        arduinoVelRevRec = parseFloat(arduinoVelRevRecInput.value);
        arduinoIntegralMax = parseFloat(arduinoIntegralMaxInput.value);
    }

    function startSimulation() {
        if (simulationRunning || isEditingTrack) return;
        if (trackPoints_pixels.length < 2) { alert("Please define a track with at least two points."); return; }
        loadParameters();
        simulationRunning = true;
        lastFrameTime = performance.now(); accumulator = 0;
        resetArduinoPIDState();
        if (!robot.trail.length || robot.trail[robot.trail.length-1].x_px === undefined) resetRobotPosition();
        animationFrameId = requestAnimationFrame(gameLoop);
        updateUIForSimulationState(true);
    }

    function stopSimulation() {
        if (!simulationRunning) return;
        simulationRunning = false;
        cancelAnimationFrame(animationFrameId);
        updateUIForSimulationState(false);
    }
    
    function resetArduinoPIDState() {
        arduinoErrorPID = 0; arduinoErrorPrevioPID = 0; arduinoTerminoProporcional = 0;
        arduinoTerminoIntegral = 0; arduinoTerminoDerivativo = 0; arduinoAjustePID = 0;
        ultimaPosicionConocidaLinea = 0;
        currentApplied_vL_mps = 0; currentApplied_vR_mps = 0;
    }

    function resetRobotPosition() {
        robot.trail = [];
        if (trackPoints_pixels.length > 0) {
            const startX_px = trackPoints_pixels[0].x; const startY_px = trackPoints_pixels[0].y;
            robot.x_m = startX_px / pixelsPerMeter; robot.y_m = startY_px / pixelsPerMeter;
            if (trackPoints_pixels.length >= 2) {
                const nextX_px = trackPoints_pixels[1].x; const nextY_px = trackPoints_pixels[1].y;
                robot.angle_rad = Math.atan2(nextY_px - startY_px, nextX_px - startX_px);
                const offsetDist_m = Math.max(0.05, currentRobotLength_m / 1.5); 
                robot.x_m -= offsetDist_m * Math.cos(robot.angle_rad);
                robot.y_m -= offsetDist_m * Math.sin(robot.angle_rad);
            } else { robot.angle_rad = -Math.PI / 2; }
        } else {
            robot.x_m = (canvas.width / 2) / pixelsPerMeter; robot.y_m = (canvas.height - 50) / pixelsPerMeter;
            robot.angle_rad = -Math.PI / 2;
        }
    }

    function resetSimulation() {
        stopSimulation(); loadParameters(); resetArduinoPIDState();
        resetRobotPosition(); 
        render(null); 
        updateUIForSimulationState(false); updateInfoDisplayDefaults();
    }

    function updateInfoDisplayDefaults() {
        errorValSpan.textContent = "0.00"; pValSpan.textContent = "0.00"; iValSpan.textContent = "0.00";
        dValSpan.textContent = "0.00"; arduinoAjusteValSpan.textContent = "N/A";
        vLeftValSpan.textContent = "0"; vRightValSpan.textContent = "0";
    }

    function handleCanvasClickForTrack(event) {
        if (!isEditingTrack) return;
        const pos_px = getMousePos(canvas, event); trackPoints_pixels.push({ x: pos_px.x, y: pos_px.y }); render(null);
    }
    function enterEditMode() {
        if (simulationRunning) stopSimulation(); isEditingTrack = true; robot.trail = [];
        updateUIForSimulationState(false); render(null); 
    }
    function clearTrack() {
        if (!isEditingTrack) return; trackPoints_pixels = []; robot.trail = []; render(null);
        editorStatusSpan.textContent = "Track cleared. Click to add new points.";
    }
    function finishEditing() {
        isEditingTrack = false; resetSimulation(); 
    }
    
    function updateUIForSimulationState(isRunning) {
        const disableSimAndEditControls = isRunning || isEditingTrack;
        startButton.disabled = disableSimAndEditControls || trackPoints_pixels.length < 2;
        stopButton.disabled = !isRunning || isEditingTrack;
        resetButton.disabled = isEditingTrack || isRunning; 
        [timeStepInput, pixelsPerMeterInput, maxRobotSpeedMPSInput, motorResponseFactorInput,
         sensorNoiseProbInput, movementPerturbFactorInput, motorDeadbandPWMInput,
         robotActualWidthInput, robotActualLengthInput, 
         sideSensorSpreadInput, sensorForwardOffsetInput,
         arduinoKpInput, arduinoKiInput, arduinoKdInput, arduinoVelBaseInput, 
         arduinoVelRecInput, arduinoVelRevRecInput, arduinoIntegralMaxInput
        ].forEach(input => { input.disabled = disableSimAndEditControls; });
        editTrackButton.disabled = isEditingTrack || isRunning; 
        clearTrackButton.disabled = !isEditingTrack || isRunning; 
        finishEditingButton.disabled = !isEditingTrack || isRunning; 
        if (isEditingTrack) {
             editorStatusSpan.textContent = "Editing: Click on canvas to add points."; canvas.classList.add('editing');
        } else {
            editorStatusSpan.textContent = `Track defined with ${trackPoints_pixels.length} points. Ready for simulation.`;
            if (trackPoints_pixels.length < 2 && !isRunning) editorStatusSpan.textContent += " (Add at least 2 points)";
            canvas.classList.remove('editing');
        }
    }

    startButton.addEventListener('click', startSimulation);
    stopButton.addEventListener('click', stopSimulation);
    resetButton.addEventListener('click', resetSimulation);
    editTrackButton.addEventListener('click', enterEditMode);
    clearTrackButton.addEventListener('click', clearTrack);
    finishEditingButton.addEventListener('click', finishEditing);
    canvas.addEventListener('click', handleCanvasClickForTrack);

    loadParameters(); 
    resetSimulation(); 
});