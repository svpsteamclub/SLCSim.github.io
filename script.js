// --- Global variables for robot graphics ---
const robotBodyImage = new Image();
const robotWheelImage = new Image();
let robotImagesLoaded = false; // A simple flag

// Define some example real-world dimensions for the wheels (top-down view)
// These could be made configurable via HTML inputs if needed later
const WHEEL_LENGTH_M = 0.3; // How long the wheel is along the robot's direction of travel (e.g., contact patch)
const WHEEL_WIDTH_M = 0.1;  // How thick the wheel is (its actual width)

// --- Function to initiate loading of robot graphics ---
function loadRobotGraphics() {
    let loadedCount = 0;
    const totalImages = 2;

    const onImageLoadOrError = () => {
        loadedCount++;
        if (loadedCount === totalImages) {
            robotImagesLoaded = true;
            console.log("Robot graphics loaded (or failed to load).");
            // Optionally, trigger a redraw if the simulation is already initialized
            // This ensures if track loads first, robot graphics still appear
            if (!simulationRunning && typeof displayCanvas !== 'undefined' && displayCanvas.getContext('2d') && currentTrackImageData) {
                 const displayCtx = displayCanvas.getContext('2d');
                 render(null); // Render the initial state with new graphics
            }
        }
    };

    robotBodyImage.onload = onImageLoadOrError;
    robotBodyImage.onerror = () => {
        console.error("Error loading robot_body.png. Using fallback.");
        onImageLoadOrError(); // Still count it to proceed
    };
    robotBodyImage.src = 'robot_body.png'; // Make sure this file exists

    robotWheelImage.onload = onImageLoadOrError;
    robotWheelImage.onerror = () => {
        console.error("Error loading robot_wheel.png. Using fallback.");
        onImageLoadOrError(); // Still count it to proceed
    };
    robotWheelImage.src = 'robot_wheel.png'; // Make sure this file exists
}


document.addEventListener('DOMContentLoaded', () => {
    const displayCanvas = document.getElementById('simulationCanvas');
    const displayCtx = displayCanvas.getContext('2d');
    const imageCanvas = document.createElement('canvas');
    const imageCtx = imageCanvas.getContext('2d', { willReadFrequently: true });

    // --- DOM Elements for UI ---
    const trackImageSelector = document.getElementById('trackImageSelector');
    const timeStepInput = document.getElementById('timeStep');
    const pixelsPerMeterDisplay = document.getElementById('pixelsPerMeterDisplay');
    const maxRobotSpeedMPSInput = document.getElementById('maxRobotSpeedMPS');
    const motorResponseFactorInput = document.getElementById('motorResponseFactor');
    const sensorNoiseProbInput = document.getElementById('sensorNoiseProb');
    const movementPerturbFactorInput = document.getElementById('movementPerturbFactor');
    const motorDeadbandPWMInput = document.getElementById('motorDeadbandPWM');
    const lineThresholdInput = document.getElementById('lineThreshold');

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

    // --- Simulation Parameters ---
    let simTimeStep, maxPhysicalSpeed_mps, currentMotorResponseFactor;
    let sensorNoiseProbability, movementPerturbationFactor, motorDeadbandPWMValue, lineThreshold;
    let currentRobotWheelbase_m, currentRobotLength_m, sensorSideSpread_m, sensorForwardProtrusion_m;
    let arduinoKp, arduinoKi, arduinoKd, arduinoVelBase, arduinoVelRec, arduinoVelRevRec, arduinoIntegralMax;

    const IMAGE_SCALE_FACTOR = 1000;
    let pixelsPerMeter = IMAGE_SCALE_FACTOR;

    let currentTrackImage = new Image();
    let currentTrackImageData = null; // Keep this global for access in loadRobotGraphics
    let currentTrackWidth_imgPx = 0;
    let currentTrackHeight_imgPx = 0;

    let arduinoErrorPID = 0, arduinoErrorPrevioPID = 0, arduinoTerminoProporcional = 0;
    let arduinoTerminoIntegral = 0, arduinoTerminoDerivativo = 0, arduinoAjustePID = 0;
    let ultimaPosicionConocidaLinea = 0;
    let currentApplied_vR_mps = 0;
    let currentApplied_vL_mps = 0;

    const sensorRadiusPixels_vis = 4;

    let robot = { x_m: 0.1, y_m: 0.1, angle_rad: 0, trail: [] };

    let simulationRunning = false; // Keep this global for access in loadRobotGraphics
    let animationFrameId;
    let accumulator = 0;
    let lastFrameTime = performance.now();

    function degreesToRadians(degrees) { return degrees * (Math.PI / 180); }
    function radiansToDegrees(radians) { return radians * (180 / Math.PI); }
    function clamp(value, min, max) { return Math.min(Math.max(value, min), max); }

    // --- Call to load robot graphics ---
    loadRobotGraphics();

    function loadTrackImage(imageUrl, imageWidthPx, imageHeightPx, startX_imgPx, startY_imgPx, startAngle_deg) {
        stopSimulation();
        currentTrackImage.onload = () => {
            currentTrackWidth_imgPx = imageWidthPx;
            currentTrackHeight_imgPx = imageHeightPx;

            displayCanvas.width = currentTrackWidth_imgPx;
            displayCanvas.height = currentTrackHeight_imgPx;
            imageCanvas.width = currentTrackWidth_imgPx;
            imageCanvas.height = currentTrackHeight_imgPx;

            imageCtx.drawImage(currentTrackImage, 0, 0, currentTrackWidth_imgPx, currentTrackHeight_imgPx);
            try {
                currentTrackImageData = imageCtx.getImageData(0, 0, currentTrackWidth_imgPx, currentTrackHeight_imgPx);
            } catch (e) {
                console.error("Error getting image data:", e);
                alert("Error loading track image data. Ensure images are served from the same origin or test on a local server if running locally.");
                currentTrackImageData = null;
                return;
            }

            robot.x_m = startX_imgPx / pixelsPerMeter;
            robot.y_m = startY_imgPx / pixelsPerMeter;
            robot.angle_rad = degreesToRadians(startAngle_deg);
            robot.trail = [];

            resetArduinoPIDState();
            render(null);
            updateUIForSimulationState(false);
        };
        currentTrackImage.onerror = () => {
            console.error(`Error loading track image: ${imageUrl}`);
            alert(`Could not load: ${imageUrl}. Ensure path is correct and file exists in the same folder as index.html.`);
            currentTrackImageData = null;
        };
        currentTrackImage.src = imageUrl;
    }

    function drawRobot() {
        displayCtx.save();
        displayCtx.translate(robot.x_m * pixelsPerMeter, robot.y_m * pixelsPerMeter);
        displayCtx.rotate(robot.angle_rad);

        const robotBodyWidthPx = currentRobotWheelbase_m * pixelsPerMeter;
        const robotBodyLengthPx = currentRobotLength_m * pixelsPerMeter;

        // --- Draw Robot Body ---
        if (robotImagesLoaded && robotBodyImage.complete && robotBodyImage.naturalWidth > 0) {
            displayCtx.drawImage(
                robotBodyImage,
                -robotBodyLengthPx / 2,
                -robotBodyWidthPx / 2,
                robotBodyLengthPx,
                robotBodyWidthPx
            );
        } else {
            displayCtx.fillStyle = 'blue';
            displayCtx.fillRect(-robotBodyLengthPx / 2, -robotBodyWidthPx / 2, robotBodyLengthPx, robotBodyWidthPx);
        }

        // --- Draw Wheels ---
        const wheelLengthPx = WHEEL_LENGTH_M * pixelsPerMeter;
        const wheelWidthPx = WHEEL_WIDTH_M * pixelsPerMeter;
        const wheelOffsetY = robotBodyWidthPx / 2;

        if (robotImagesLoaded && robotWheelImage.complete && robotWheelImage.naturalWidth > 0) {
            // Left Wheel
            displayCtx.drawImage(
                robotWheelImage,
                -wheelLengthPx / 2,
                -wheelOffsetY - wheelWidthPx / 2,
                wheelLengthPx,
                wheelWidthPx
            );
            // Right Wheel
            displayCtx.drawImage(
                robotWheelImage,
                -wheelLengthPx / 2,
                wheelOffsetY - wheelWidthPx / 2,
                wheelLengthPx,
                wheelWidthPx
            );
        } else {
            displayCtx.fillStyle = '#555555';
            // Left wheel
            displayCtx.fillRect(-wheelLengthPx / 2, -wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
            // Right wheel
            displayCtx.fillRect(-wheelLengthPx / 2, wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
        }

        // --- Direction Indicator ---
        displayCtx.fillStyle = 'lightblue';
        displayCtx.beginPath();
        const indicatorTipX = robotBodyLengthPx / 2 + 3;
        const indicatorBaseX = robotBodyLengthPx / 2 - Math.min(8, robotBodyLengthPx * 0.1);
        const indicatorBaseSpread = robotBodyWidthPx / 4; // Adjusted for potentially smaller indicator
        displayCtx.moveTo(indicatorTipX, 0);
        displayCtx.lineTo(indicatorBaseX, -indicatorBaseSpread / 2);
        displayCtx.lineTo(indicatorBaseX, indicatorBaseSpread / 2);
        displayCtx.closePath(); displayCtx.fill();

        displayCtx.restore();

        if (robot.trail.length > 1) {
            displayCtx.beginPath(); displayCtx.strokeStyle = 'rgba(0, 0, 255, 0.3)'; displayCtx.lineWidth = 2;
            displayCtx.moveTo(robot.trail[0].x_m * pixelsPerMeter, robot.trail[0].y_m * pixelsPerMeter);
            for (let i = 1; i < robot.trail.length; i++) {
                displayCtx.lineTo(robot.trail[i].x_m * pixelsPerMeter, robot.trail[i].y_m * pixelsPerMeter);
            }
            displayCtx.stroke();
        }
    }

    function getSensorPositions_imagePx() {
        const sensorLineCenterX_m = robot.x_m + sensorForwardProtrusion_m * Math.cos(robot.angle_rad);
        const sensorLineCenterY_m = robot.y_m + sensorForwardProtrusion_m * Math.sin(robot.angle_rad);
        const perpendicularAngle = robot.angle_rad - Math.PI / 2;
        const lX_m = sensorLineCenterX_m + sensorSideSpread_m * Math.cos(perpendicularAngle);
        const lY_m = sensorLineCenterY_m + sensorSideSpread_m * Math.sin(perpendicularAngle);
        const cX_m = sensorLineCenterX_m; const cY_m = sensorLineCenterY_m;
        const rX_m = sensorLineCenterX_m - sensorSideSpread_m * Math.cos(perpendicularAngle);
        const rY_m = sensorLineCenterY_m - sensorSideSpread_m * Math.sin(perpendicularAngle);
        return {
            left:   { x: Math.round(lX_m * pixelsPerMeter), y: Math.round(lY_m * pixelsPerMeter) },
            center: { x: Math.round(cX_m * pixelsPerMeter), y: Math.round(cY_m * pixelsPerMeter) },
            right:  { x: Math.round(rX_m * pixelsPerMeter), y: Math.round(rY_m * pixelsPerMeter) }
        };
    }

    function drawSensors(sensorStates) {
        const positions_img_px = getSensorPositions_imagePx();
        drawSensor(positions_img_px.left, sensorStates.left);
        drawSensor(positions_img_px.center, sensorStates.center);
        drawSensor(positions_img_px.right, sensorStates.right);
    }

    function drawSensor(pos_px, isOnLine) {
        displayCtx.beginPath(); displayCtx.arc(pos_px.x, pos_px.y, sensorRadiusPixels_vis, 0, 2 * Math.PI);
        displayCtx.fillStyle = isOnLine ? 'red' : 'gray'; displayCtx.fill();
        displayCtx.strokeStyle = 'black'; displayCtx.lineWidth = 1; displayCtx.stroke();
    }

    function isPixelOnLine(x_img_px, y_img_px) {
        if (!currentTrackImageData || x_img_px < 0 || x_img_px >= currentTrackWidth_imgPx || y_img_px < 0 || y_img_px >= currentTrackHeight_imgPx) {
            return false;
        }
        const R_INDEX = (y_img_px * currentTrackWidth_imgPx + x_img_px) * 4;
        const r = currentTrackImageData.data[R_INDEX];
        const g = currentTrackImageData.data[R_INDEX + 1];
        const b = currentTrackImageData.data[R_INDEX + 2];
        const brightness = (r + g + b) / 3;
        return brightness < lineThreshold;
    }

    function fixedUpdate(dt_s) {
        const sensorPositions_img_px = getSensorPositions_imagePx();
        let s_der_active = isPixelOnLine(sensorPositions_img_px.right.x, sensorPositions_img_px.right.y);
        let s_cen_active = isPixelOnLine(sensorPositions_img_px.center.x, sensorPositions_img_px.center.y);
        let s_izq_active = isPixelOnLine(sensorPositions_img_px.left.x, sensorPositions_img_px.left.y);

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

        robot.trail.push({ x_m: robot.x_m, y_m: robot.y_m });
        if (robot.trail.length > 500) robot.trail.shift();

        const boundaryMargin_m = Math.max(currentRobotWheelbase_m, currentRobotLength_m) / 2;
        if (robot.x_m < -boundaryMargin_m || robot.x_m * pixelsPerMeter > displayCanvas.width + boundaryMargin_m * pixelsPerMeter ||
            robot.y_m < -boundaryMargin_m || robot.y_m * pixelsPerMeter > displayCanvas.height + boundaryMargin_m * pixelsPerMeter) {
            stopSimulation();
        }
        return { left: s_izq_active, center: s_cen_active, right: s_der_active };
    }

    function render(sensorStates) {
        displayCtx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);
        if (currentTrackImage.complete && currentTrackImage.src && currentTrackWidth_imgPx > 0) {
            displayCtx.drawImage(currentTrackImage, 0, 0, displayCanvas.width, displayCanvas.height);
        } else {
            displayCtx.fillStyle = '#eee';
            displayCtx.fillRect(0,0, displayCanvas.width, displayCanvas.height);
            displayCtx.fillStyle = 'black';
            displayCtx.textAlign = 'center';
            displayCtx.fillText("Loading track or no track selected...", displayCanvas.width/2, displayCanvas.height/2);
        }

        // Ensure drawRobot and drawSensors are called with displayCtx
        if (simulationRunning || (currentTrackImageData && !simulationRunning) ) {
            drawRobot(); // drawRobot uses displayCtx internally
            if (simulationRunning && sensorStates) drawSensors(sensorStates); // drawSensors uses displayCtx
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
        pixelsPerMeterDisplay.value = IMAGE_SCALE_FACTOR.toFixed(0);
        pixelsPerMeter = IMAGE_SCALE_FACTOR;
        maxPhysicalSpeed_mps = parseFloat(maxRobotSpeedMPSInput.value);
        currentMotorResponseFactor = parseFloat(motorResponseFactorInput.value);
        currentMotorResponseFactor = clamp(currentMotorResponseFactor, 0.01, 1.0);
        sensorNoiseProbability = parseFloat(sensorNoiseProbInput.value);
        sensorNoiseProbability = clamp(sensorNoiseProbability, 0, 1);
        movementPerturbationFactor = parseFloat(movementPerturbFactorInput.value);
        movementPerturbationFactor = clamp(movementPerturbationFactor, 0, 0.5);
        motorDeadbandPWMValue = parseFloat(motorDeadbandPWMInput.value);
        motorDeadbandPWMValue = clamp(motorDeadbandPWMValue, 0, 50);
        lineThreshold = parseInt(lineThresholdInput.value);
        lineThreshold = clamp(lineThreshold, 0, 255);

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
        if (simulationRunning) return;
        if (!currentTrackImageData) { alert("Please select a track image."); return; }
        loadParameters();
        simulationRunning = true;
        lastFrameTime = performance.now(); accumulator = 0;
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

    function resetSimulation() {
        stopSimulation();
        loadParameters();
        const selectedOption = trackImageSelector.options[trackImageSelector.selectedIndex];
        if (selectedOption && selectedOption.value) {
            const imageUrl = selectedOption.value;
            const imgWidth = parseInt(selectedOption.dataset.width);
            const imgHeight = parseInt(selectedOption.dataset.height);
            const startX = parseInt(selectedOption.dataset.startX);
            const startY = parseInt(selectedOption.dataset.startY);
            const startAngle = parseFloat(selectedOption.dataset.startAngle);
            loadTrackImage(imageUrl, imgWidth, imgHeight, startX, startY, startAngle);
        } else if (trackImageSelector.options.length > 0) {
             trackImageSelector.selectedIndex = 0;
             trackImageSelector.dispatchEvent(new Event('change'));
        } else {
            console.warn("No track selected or available for reset.");
            displayCtx.clearRect(0,0,displayCanvas.width, displayCanvas.height);
            displayCtx.fillStyle = '#eee';
            displayCtx.fillRect(0,0, displayCanvas.width, displayCanvas.height);
            displayCtx.fillStyle = 'black';
            displayCtx.textAlign = 'center';
            displayCtx.fillText("Select a track to begin.", displayCanvas.width/2, displayCanvas.height/2);
        }
        updateInfoDisplayDefaults();
    }

    function updateInfoDisplayDefaults() {
        errorValSpan.textContent = "0.00"; pValSpan.textContent = "0.00"; iValSpan.textContent = "0.00";
        dValSpan.textContent = "0.00"; arduinoAjusteValSpan.textContent = "N/A";
        vLeftValSpan.textContent = "0"; vRightValSpan.textContent = "0";
    }

    function updateUIForSimulationState(isRunning) {
        const disableAllInputs = isRunning;
        startButton.disabled = isRunning || !currentTrackImageData;
        stopButton.disabled = !isRunning;
        resetButton.disabled = isRunning;
        [timeStepInput, pixelsPerMeterDisplay, maxRobotSpeedMPSInput, motorResponseFactorInput,
         sensorNoiseProbInput, movementPerturbFactorInput, motorDeadbandPWMInput, lineThresholdInput,
         robotActualWidthInput, robotActualLengthInput,
         sideSensorSpreadInput, sensorForwardOffsetInput,
         arduinoKpInput, arduinoKiInput, arduinoKdInput, arduinoVelBaseInput,
         arduinoVelRecInput, arduinoVelRevRecInput, arduinoIntegralMaxInput,
         trackImageSelector
        ].forEach(input => { input.disabled = disableAllInputs; });
    }

    startButton.addEventListener('click', startSimulation);
    stopButton.addEventListener('click', stopSimulation);
    resetButton.addEventListener('click', resetSimulation);
    trackImageSelector.addEventListener('change', (event) => {
        const selectedOption = event.target.options[event.target.selectedIndex];
        const imageUrl = selectedOption.value;
        const imgWidth = parseInt(selectedOption.dataset.width);
        const imgHeight = parseInt(selectedOption.dataset.height);
        const startX = parseInt(selectedOption.dataset.startX);
        const startY = parseInt(selectedOption.dataset.startY);
        const startAngle = parseFloat(selectedOption.dataset.startAngle);
        loadTrackImage(imageUrl, imgWidth, imgHeight, startX, startY, startAngle);
    });

    loadParameters();
    if (trackImageSelector.options.length > 0) {
        trackImageSelector.dispatchEvent(new Event('change')); // Trigger initial load
    } else {
        displayCanvas.width = 800; displayCanvas.height = 600;
        render(null);
        updateUIForSimulationState(false);
        updateInfoDisplayDefaults();
        alert("No tracks defined. Add <option> tags to #trackImageSelector in index.html with data attributes.");
    }
});