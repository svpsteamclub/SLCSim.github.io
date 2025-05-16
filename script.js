// --- Global variables for robot graphics ---
const robotBodyImage = new Image();
const robotWheelImage = new Image();
let robotImagesLoaded = false; 

// --- Global variable for watermark graphic ---
const watermarkImage = new Image();
let watermarkImageLoaded = false;

const WHEEL_LENGTH_M = 0.07;
const WHEEL_WIDTH_M = 0.03;

// --- Constants for Bar Display ---
const MAX_BAR_HEIGHT_PX = 50; // Corresponds to .bar-track height in CSS

// Max expected ABSOLUTE values for scaling bars (tune these as needed)
let currentMaxValError = 2.5;
let currentMaxValPTerm = 150;
let currentMaxValITerm = 50; // Will be updated from arduinoIntegralMaxInput
let currentMaxValDTerm = 150;
let currentMaxValAdjPID = 255; // Assuming PID output is often scaled similarly to PWM
const MAX_VAL_PWM_BAR = 255;


function checkAndRenderInitialState() {
    if (currentTrackImageData && robotImagesLoaded && watermarkImageLoaded) {
        console.log("All initial assets ready. Rendering initial state.");
        render(null); 
        if (!simulationRunning) {
            updateUIForSimulationState(false);
        }
    }
}

function loadRobotGraphics() {
    let loadedCount = 0;
    const totalImages = 2;
    const onImageLoadOrError = () => {
        loadedCount++;
        if (loadedCount === totalImages) {
            robotImagesLoaded = true; 
            console.log("Robot graphics loading attempt complete.");
            checkAndRenderInitialState(); 
        }
    };
    robotBodyImage.onload = onImageLoadOrError;
    robotBodyImage.onerror = () => { console.error("Error loading robot_body.png."); onImageLoadOrError(); };
    robotBodyImage.src = 'robot_body.png';
    robotWheelImage.onload = onImageLoadOrError;
    robotWheelImage.onerror = () => { console.error("Error loading robot_wheel.png."); onImageLoadOrError(); };
    robotWheelImage.src = 'robot_wheel.png';
}

function loadWatermarkGraphic() {
    watermarkImage.onload = () => {
        console.log("Watermark image loaded.");
        watermarkImageLoaded = true; 
        checkAndRenderInitialState(); 
    };
    watermarkImage.onerror = () => {
        console.error("Error loading SVPSTEAM_Club.png for watermark.");
        watermarkImageLoaded = true; 
        checkAndRenderInitialState(); 
    };
    watermarkImage.src = 'SVPSTEAM_Club.png';
}


document.addEventListener('DOMContentLoaded', () => {
    const displayCanvas = document.getElementById('simulationCanvas');
    const displayCtx = displayCanvas.getContext('2d');
    const imageCanvas = document.createElement('canvas');
    const imageCtx = imageCanvas.getContext('2d', { willReadFrequently: true });
    
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
    const sensorDiameterInput = document.getElementById('sensorDiameterInput'); 
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

    // --- DOM Elements for Text Values ---
    const errorValSpan = document.getElementById('errorVal');
    const pValSpan = document.getElementById('pVal');
    const iValSpan = document.getElementById('iVal');
    const dValSpan = document.getElementById('dVal');
    const arduinoAjusteValSpan = document.getElementById('arduinoAjusteVal');
    const vLeftValSpan = document.getElementById('vLeftVal');
    const vRightValSpan = document.getElementById('vRightVal');

    // --- DOM Elements for Bars ---
    const errorBar = document.getElementById('errorBar');
    const pBar = document.getElementById('pBar');
    const iBar = document.getElementById('iBar');
    const dBar = document.getElementById('dBar');
    const adjPIDBar = document.getElementById('adjPIDBar');
    const vLeftBar = document.getElementById('vLeftBar');
    const vRightBar = document.getElementById('vRightBar');

    let simTimeStep, maxPhysicalSpeed_mps, currentMotorResponseFactor;
    let sensorNoiseProbability, movementPerturbationFactor, motorDeadbandPWMValue, lineThreshold;
    let currentRobotWheelbase_m, currentRobotLength_m, sensorSideSpread_m, sensorForwardProtrusion_m, currentSensorDiameter_m; 
    const IMAGE_SCALE_FACTOR = 1000;
    let pixelsPerMeter = IMAGE_SCALE_FACTOR;
    let currentTrackImage = new Image();
    let currentTrackImageData = null; 
    let currentTrackWidth_imgPx = 0;
    let currentTrackHeight_imgPx = 0;
    let arduinoErrorPID = 0, arduinoErrorPrevioPID = 0, arduinoTerminoProporcional = 0;
    let arduinoTerminoIntegral = 0, arduinoTerminoDerivativo = 0, arduinoAjustePID = 0;
    let ultimaPosicionConocidaLinea = 0;
    let currentApplied_vR_mps = 0;
    let currentApplied_vL_mps = 0;
    let robot = { x_m: 0.1, y_m: 0.1, angle_rad: 0, centerTrail: [], leftWheelTrail: [], rightWheelTrail: [] };
    let simulationRunning = false; 
    let animationFrameId;
    let accumulator = 0;
    let lastFrameTime = performance.now();

    function degreesToRadians(degrees) { return degrees * (Math.PI / 180); }
    function radiansToDegrees(radians) { return radians * (180 / Math.PI); }
    function clamp(value, min, max) { return Math.min(Math.max(value, min), max); }

    loadRobotGraphics();
    loadWatermarkGraphic();

    function loadTrackImage(imageUrl, imageWidthPx, imageHeightPx, startX_imgPx, startY_imgPx, startAngle_deg) {
        stopSimulation();
        currentTrackImageData = null; 
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
                alert("Error loading track image data.");
                currentTrackImageData = null; 
                return;
            }
            console.log("Track image loaded and data processed.");
            robot.x_m = startX_imgPx / pixelsPerMeter;
            robot.y_m = startY_imgPx / pixelsPerMeter;
            robot.angle_rad = degreesToRadians(startAngle_deg);
            robot.centerTrail = []; robot.leftWheelTrail = []; robot.rightWheelTrail = [];
            resetArduinoPIDState();
            checkAndRenderInitialState(); 
            updateUIForSimulationState(false); 
        };
        currentTrackImage.onerror = () => {
            console.error(`Error loading track image: ${imageUrl}`);
            alert(`Could not load: ${imageUrl}.`);
            currentTrackImageData = null;
            checkAndRenderInitialState(); 
        };
        currentTrackImage.src = imageUrl;
    }

    function drawRobot() {
        displayCtx.save();
        displayCtx.translate(robot.x_m * pixelsPerMeter, robot.y_m * pixelsPerMeter);
        displayCtx.rotate(robot.angle_rad);
        const robotBodyWidthPx = currentRobotWheelbase_m * pixelsPerMeter;
        const robotBodyLengthPx = currentRobotLength_m * pixelsPerMeter;

        if (robotImagesLoaded && robotBodyImage.complete && robotBodyImage.naturalWidth > 0) {
            displayCtx.drawImage(robotBodyImage, -robotBodyLengthPx / 2, -robotBodyWidthPx / 2, robotBodyLengthPx, robotBodyWidthPx);
        } else {
            displayCtx.fillStyle = 'blue';
            displayCtx.fillRect(-robotBodyLengthPx / 2, -robotBodyWidthPx / 2, robotBodyLengthPx, robotBodyWidthPx);
        }
        const wheelLengthPx = WHEEL_LENGTH_M * pixelsPerMeter;
        const wheelWidthPx = WHEEL_WIDTH_M * pixelsPerMeter;
        const wheelOffsetY = robotBodyWidthPx / 2;
        if (robotImagesLoaded && robotWheelImage.complete && robotWheelImage.naturalWidth > 0) {
            displayCtx.drawImage(robotWheelImage, -wheelLengthPx / 2, -wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
            displayCtx.drawImage(robotWheelImage, -wheelLengthPx / 2, wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
        } else {
            displayCtx.fillStyle = '#555555';
            displayCtx.fillRect(-wheelLengthPx / 2, -wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
            displayCtx.fillRect(-wheelLengthPx / 2, wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
        }
        displayCtx.fillStyle = 'lightblue';
        displayCtx.beginPath();
        const indicatorTipX = robotBodyLengthPx / 2 + 3;
        const indicatorBaseX = robotBodyLengthPx / 2 - Math.min(8, robotBodyLengthPx * 0.1);
        const indicatorBaseSpread = robotBodyWidthPx / 4; 
        displayCtx.moveTo(indicatorTipX, 0);
        displayCtx.lineTo(indicatorBaseX, -indicatorBaseSpread / 2);
        displayCtx.lineTo(indicatorBaseX, indicatorBaseSpread / 2);
        displayCtx.closePath(); displayCtx.fill();
        displayCtx.restore(); 
        if (robot.centerTrail.length > 1) { /* ... trail drawing ... */ }
        if (robot.leftWheelTrail.length > 1) { /* ... trail drawing ... */ }
        if (robot.rightWheelTrail.length > 1) { /* ... trail drawing ... */ }
    }
    // Condensed trail drawing for brevity
    function drawRobot() {
        displayCtx.save();
        displayCtx.translate(robot.x_m * pixelsPerMeter, robot.y_m * pixelsPerMeter);
        displayCtx.rotate(robot.angle_rad);
        const robotBodyWidthPx = currentRobotWheelbase_m * pixelsPerMeter;
        const robotBodyLengthPx = currentRobotLength_m * pixelsPerMeter;
        if (robotImagesLoaded && robotBodyImage.complete && robotBodyImage.naturalWidth > 0) {
            displayCtx.drawImage(robotBodyImage, -robotBodyLengthPx / 2, -robotBodyWidthPx / 2, robotBodyLengthPx, robotBodyWidthPx);
        } else { displayCtx.fillStyle = 'blue'; displayCtx.fillRect(-robotBodyLengthPx / 2, -robotBodyWidthPx / 2, robotBodyLengthPx, robotBodyWidthPx); }
        const wheelLengthPx = WHEEL_LENGTH_M * pixelsPerMeter; const wheelWidthPx = WHEEL_WIDTH_M * pixelsPerMeter; const wheelOffsetY = robotBodyWidthPx / 2;
        if (robotImagesLoaded && robotWheelImage.complete && robotWheelImage.naturalWidth > 0) {
            displayCtx.drawImage(robotWheelImage, -wheelLengthPx / 2, -wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
            displayCtx.drawImage(robotWheelImage, -wheelLengthPx / 2, wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx);
        } else { displayCtx.fillStyle = '#555555'; displayCtx.fillRect(-wheelLengthPx / 2, -wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx); displayCtx.fillRect(-wheelLengthPx / 2, wheelOffsetY - wheelWidthPx / 2, wheelLengthPx, wheelWidthPx); }
        displayCtx.fillStyle = 'lightblue'; displayCtx.beginPath(); const indicatorTipX = robotBodyLengthPx / 2 + 3; const indicatorBaseX = robotBodyLengthPx / 2 - Math.min(8, robotBodyLengthPx * 0.1); const indicatorBaseSpread = robotBodyWidthPx / 4;  displayCtx.moveTo(indicatorTipX, 0); displayCtx.lineTo(indicatorBaseX, -indicatorBaseSpread / 2); displayCtx.lineTo(indicatorBaseX, indicatorBaseSpread / 2); displayCtx.closePath(); displayCtx.fill();
        displayCtx.restore();
        const drawTrail = (trail, color, width) => {
            if (trail.length > 1) {
                displayCtx.beginPath(); displayCtx.strokeStyle = color; displayCtx.lineWidth = width;
                displayCtx.moveTo(trail[0].x_m * pixelsPerMeter, trail[0].y_m * pixelsPerMeter);
                for (let i = 1; i < trail.length; i++) { displayCtx.lineTo(trail[i].x_m * pixelsPerMeter, trail[i].y_m * pixelsPerMeter); }
                displayCtx.stroke();
            }
        };
        drawTrail(robot.centerTrail, 'rgba(0, 0, 255, 0.3)', 20);
        drawTrail(robot.leftWheelTrail, 'rgba(255, 0, 0, 0.4)', 15);
        drawTrail(robot.rightWheelTrail, 'rgba(0, 255, 0, 0.4)', 15);
    }


    function getSensorPositions_imagePx() { /* ... same ... */ return {left:{x:0,y:0},center:{x:0,y:0},right:{x:0,y:0}}; }
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
    function drawSensors(sensorStates) { /* ... same ... */ }
    function drawSensors(sensorStates) {
        const positions_img_px = getSensorPositions_imagePx();
        drawSensor(positions_img_px.left, sensorStates.left);
        drawSensor(positions_img_px.center, sensorStates.center);
        drawSensor(positions_img_px.right, sensorStates.right);
    }
    function drawSensor(pos_px, isOnLine) { /* ... same ... */ }
     function drawSensor(pos_px, isOnLine) {
        const sensorRadiusPx = (currentSensorDiameter_m / 2) * pixelsPerMeter; 
        displayCtx.beginPath(); 
        displayCtx.arc(pos_px.x, pos_px.y, Math.max(1, sensorRadiusPx), 0, 2 * Math.PI); 
        displayCtx.fillStyle = isOnLine ? 'green' : 'gray'; 
        displayCtx.fill();
        displayCtx.strokeStyle = 'black'; 
        displayCtx.lineWidth = 1; 
        displayCtx.stroke();
    }
    function isPixelOnLine(x_img_px, y_img_px) { /* ... same ... */ return false; }
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

    function updateBar(barElement, value, maxValue, valueTextElement) {
        let absValue = 0;
        let heightPercentage = 0;

        // Check if the text display (valueTextElement) shows a non-numeric value like "REC" or "N/A"
        if (valueTextElement && isNaN(parseFloat(valueTextElement.textContent))) {
            heightPercentage = 0; // Set bar to zero for "REC", "N/A", etc.
        } else if (typeof value === 'number' && !isNaN(value)) {
            absValue = Math.abs(value);
            if (maxValue > 0.00001) { // Avoid division by zero or tiny max values
                heightPercentage = Math.min(100, (absValue / maxValue) * 100);
            }
        }
        
        if (isNaN(heightPercentage)) { // Final safety check
            heightPercentage = 0;
        }
        barElement.style.height = `${heightPercentage}%`;
    }

    function fixedUpdate(dt_s) {
        const sensorPositions_img_px = getSensorPositions_imagePx();
        let s_der_active = isPixelOnLine(sensorPositions_img_px.right.x, sensorPositions_img_px.right.y);
        let s_cen_active = isPixelOnLine(sensorPositions_img_px.center.x, sensorPositions_img_px.center.y);
        let s_izq_active = isPixelOnLine(sensorPositions_img_px.left.x, sensorPositions_img_px.left.y);

        if (simulationRunning && sensorNoiseProbability > 0) { /* ... noise ... */ }

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
        updateBar(errorBar, arduinoErrorPID, currentMaxValError, errorValSpan);

        let velRawMotorA_target = 0; let velRawMotorB_target = 0;
        let dirA_adelante = true; let dirB_adelante = true;

        if (lineaPerdida) {
            arduinoTerminoIntegral = 0; arduinoErrorPrevioPID = 0; arduinoAjustePID = 0; arduinoTerminoProporcional = 0; arduinoTerminoDerivativo = 0;
            pValSpan.textContent = "REC"; iValSpan.textContent = "REC"; dValSpan.textContent = "REC";
            updateBar(pBar, 0, currentMaxValPTerm, pValSpan);
            updateBar(iBar, 0, currentMaxValITerm, iValSpan);
            updateBar(dBar, 0, currentMaxValDTerm, dValSpan);
            // Recovery speeds
            if (ultimaPosicionConocidaLinea === 1) { /* ... recovery speeds ... */ }
            else if (ultimaPosicionConocidaLinea === 2) { /* ... recovery speeds ... */ }
            else { /* ... recovery speeds ... */ }
            if (ultimaPosicionConocidaLinea === 1) { velRawMotorA_target = arduinoVelRec; dirA_adelante = true; velRawMotorB_target = arduinoVelRevRec; dirB_adelante = false;}
            else if (ultimaPosicionConocidaLinea === 2) { velRawMotorA_target = arduinoVelRevRec; dirA_adelante = false; velRawMotorB_target = arduinoVelRec; dirB_adelante = true; }
            else { velRawMotorA_target = arduinoVelRec; dirA_adelante = true; velRawMotorB_target = arduinoVelRec; dirB_adelante = true; }

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
            updateBar(pBar, arduinoTerminoProporcional, currentMaxValPTerm, pValSpan);
            updateBar(iBar, arduinoTerminoIntegral, currentMaxValITerm, iValSpan);
            updateBar(dBar, arduinoTerminoDerivativo, currentMaxValDTerm, dValSpan);
        }
        arduinoAjusteValSpan.textContent = arduinoAjustePID.toFixed(2);
        updateBar(adjPIDBar, arduinoAjustePID, currentMaxValAdjPID, arduinoAjusteValSpan);
        
        let finalVelRawMotorA = velRawMotorA_target; /* ... deadband and clamping ... */
        let finalVelRawMotorB = velRawMotorB_target; /* ... deadband and clamping ... */
        if (Math.abs(velRawMotorA_target) < motorDeadbandPWMValue) finalVelRawMotorA = 0;
        if (Math.abs(velRawMotorB_target) < motorDeadbandPWMValue) finalVelRawMotorB = 0;
        finalVelRawMotorA = clamp(finalVelRawMotorA, 0, 255);
        finalVelRawMotorB = clamp(finalVelRawMotorB, 0, 255);

        vLeftValSpan.textContent = Math.round(finalVelRawMotorB);
        vRightValSpan.textContent = Math.round(finalVelRawMotorA);
        updateBar(vLeftBar, finalVelRawMotorB, MAX_VAL_PWM_BAR, vLeftValSpan);
        updateBar(vRightBar, finalVelRawMotorA, MAX_VAL_PWM_BAR, vRightValSpan);

        let target_vR_mps = (dirA_adelante ? 1 : -1) * (finalVelRawMotorA / 255.0) * maxPhysicalSpeed_mps; /* ... motor response ... */
        let target_vL_mps = (dirB_adelante ? 1 : -1) * (finalVelRawMotorB / 255.0) * maxPhysicalSpeed_mps; /* ... motor response ... */
        currentApplied_vR_mps += (target_vR_mps - currentApplied_vR_mps) * currentMotorResponseFactor;
        currentApplied_vL_mps += (target_vL_mps - currentApplied_vL_mps) * currentMotorResponseFactor;

        const L_m = currentRobotWheelbase_m; /* ... robot movement ... */
        let d_theta_rad = 0; if (L_m > 0.001) { d_theta_rad = -(currentApplied_vR_mps - currentApplied_vL_mps) / L_m * dt_s; }
        let linear_displacement_m = (currentApplied_vR_mps + currentApplied_vL_mps) / 2.0 * dt_s;
        if (simulationRunning && movementPerturbationFactor > 0) { /* ... perturbation ... */ }
        robot.angle_rad += d_theta_rad; robot.angle_rad = Math.atan2(Math.sin(robot.angle_rad), Math.cos(robot.angle_rad)); 
        robot.x_m += linear_displacement_m * Math.cos(robot.angle_rad); robot.y_m += linear_displacement_m * Math.sin(robot.angle_rad);
        robot.centerTrail.push({ x_m: robot.x_m, y_m: robot.y_m }); if (robot.centerTrail.length > 500) robot.centerTrail.shift();
        const halfWheelbase_m = currentRobotWheelbase_m / 2; const sinAngle = Math.sin(robot.angle_rad); const cosAngle = Math.cos(robot.angle_rad);
        const x_lw_m = robot.x_m + halfWheelbase_m * sinAngle; const y_lw_m = robot.y_m - halfWheelbase_m * cosAngle; robot.leftWheelTrail.push({ x_m: x_lw_m, y_m: y_lw_m }); if (robot.leftWheelTrail.length > 500) robot.leftWheelTrail.shift();
        const x_rw_m = robot.x_m - halfWheelbase_m * sinAngle; const y_rw_m = robot.y_m + halfWheelbase_m * cosAngle; robot.rightWheelTrail.push({ x_m: x_rw_m, y_m: y_rw_m }); if (robot.rightWheelTrail.length > 500) robot.rightWheelTrail.shift();
        const boundaryMargin_m = Math.max(currentRobotWheelbase_m, currentRobotLength_m) / 2; if (robot.x_m < -boundaryMargin_m || robot.x_m * pixelsPerMeter > displayCanvas.width + boundaryMargin_m * pixelsPerMeter || robot.y_m < -boundaryMargin_m || robot.y_m * pixelsPerMeter > displayCanvas.height + boundaryMargin_m * pixelsPerMeter) { stopSimulation(); }

        return { left: s_izq_active, center: s_cen_active, right: s_der_active };
    }

    function render(sensorStates) { /* ... same logic ... */ }
    function render(sensorStates) {
        displayCtx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);
        if (currentTrackImageData && currentTrackImage.complete && currentTrackImage.naturalWidth > 0) {
            displayCtx.drawImage(currentTrackImage, 0, 0, displayCanvas.width, displayCanvas.height);
            if (watermarkImageLoaded && watermarkImage.complete && watermarkImage.naturalWidth > 0) {
                const watermarkAspectRatio = watermarkImage.naturalWidth / watermarkImage.naturalHeight;
                let watermarkWidth = displayCanvas.width * 0.3; let watermarkHeight = watermarkWidth / watermarkAspectRatio;
                if (watermarkHeight > displayCanvas.height * 0.3) { watermarkHeight = displayCanvas.height * 0.3; watermarkWidth = watermarkHeight * watermarkAspectRatio; }
                if (watermarkWidth > displayCanvas.width * 0.3) { watermarkWidth = displayCanvas.width * 0.3; watermarkHeight = watermarkWidth / watermarkAspectRatio; }
                const watermarkX = (displayCanvas.width - watermarkWidth) / 2; const watermarkY = (displayCanvas.height - watermarkHeight) / 2;
                displayCtx.save(); displayCtx.globalAlpha = 0.2; displayCtx.drawImage(watermarkImage, watermarkX, watermarkY, watermarkWidth, watermarkHeight); displayCtx.restore();
            }
        } else { /* ... fallback for no track ... */ 
            displayCtx.fillStyle = '#eee'; displayCtx.fillRect(0, 0, displayCanvas.width, displayCanvas.height); displayCtx.fillStyle = 'black'; displayCtx.textAlign = 'center';
            let message = "Loading assets...";
            if (trackImageSelector.value && !currentTrackImageData && currentTrackImage.src.endsWith(trackImageSelector.value) && !currentTrackImage.complete) { message = "Loading track...";}
            else if (!trackImageSelector.value) { message = "No track selected.";}
            else if (currentTrackImageData === null && currentTrackImage.src) { message = "Error loading track. Please check console.";}
            displayCtx.fillText(message, displayCanvas.width / 2, displayCanvas.height / 2);
        }
        if (currentTrackImageData !== undefined && robotImagesLoaded) { 
             if(currentRobotLength_m > 0 && currentRobotWheelbase_m > 0) { 
                drawRobot();
                if (simulationRunning && sensorStates) drawSensors(sensorStates);
            }
        }
    }

    function gameLoop(currentTime) { /* ... same ... */ }
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
        simTimeStep = parseFloat(timeStepInput.value); /* ... other params ... */
        pixelsPerMeterDisplay.value = IMAGE_SCALE_FACTOR.toFixed(0); pixelsPerMeter = IMAGE_SCALE_FACTOR;
        maxPhysicalSpeed_mps = parseFloat(maxRobotSpeedMPSInput.value);
        currentMotorResponseFactor = parseFloat(motorResponseFactorInput.value); currentMotorResponseFactor = clamp(currentMotorResponseFactor, 0.01, 1.0);
        sensorNoiseProbability = parseFloat(sensorNoiseProbInput.value); sensorNoiseProbability = clamp(sensorNoiseProbability, 0, 1);
        movementPerturbationFactor = parseFloat(movementPerturbFactorInput.value); movementPerturbationFactor = clamp(movementPerturbationFactor, 0, 0.5);
        motorDeadbandPWMValue = parseFloat(motorDeadbandPWMInput.value); motorDeadbandPWMValue = clamp(motorDeadbandPWMValue, 0, 50);
        lineThreshold = parseInt(lineThresholdInput.value); lineThreshold = clamp(lineThreshold, 0, 255);
        currentRobotWheelbase_m = parseFloat(robotActualWidthInput.value); currentRobotLength_m = parseFloat(robotActualLengthInput.value);
        sensorSideSpread_m = parseFloat(sideSensorSpreadInput.value); sensorForwardProtrusion_m = parseFloat(sensorForwardOffsetInput.value);
        currentSensorDiameter_m = parseFloat(sensorDiameterInput.value); 
        arduinoKp = parseFloat(arduinoKpInput.value); arduinoKi = parseFloat(arduinoKiInput.value); arduinoKd = parseFloat(arduinoKdInput.value);
        arduinoVelBase = parseFloat(arduinoVelBaseInput.value); arduinoVelRec = parseFloat(arduinoVelRecInput.value);
        arduinoVelRevRec = parseFloat(arduinoVelRevRecInput.value); 
        arduinoIntegralMax = parseFloat(arduinoIntegralMaxInput.value);

        // Update max value for I term bar scaling
        currentMaxValITerm = arduinoIntegralMax;
        if (isNaN(currentMaxValITerm) || currentMaxValITerm <= 0.00001) { // Ensure it's a positive number for scaling
            currentMaxValITerm = 50; // Default if invalid or zero (to avoid division by zero for bar)
        }
    }

    function startSimulation() { /* ... same ... */ }
    function startSimulation() {
        if (simulationRunning) return;
        if (!currentTrackImageData) { alert("Please select and wait for a track image to load."); return; }
        loadParameters();
        simulationRunning = true;
        lastFrameTime = performance.now(); accumulator = 0;
        animationFrameId = requestAnimationFrame(gameLoop);
        updateUIForSimulationState(true);
    }
    function stopSimulation() { /* ... same ... */ }
    function stopSimulation() {
        if (!simulationRunning) return;
        simulationRunning = false;
        cancelAnimationFrame(animationFrameId);
        updateUIForSimulationState(false);
    }
    function resetArduinoPIDState() { /* ... same ... */ }
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
            /* ... load track ... */
            const imageUrl = selectedOption.value;
            const imgWidth = parseInt(selectedOption.dataset.width); const imgHeight = parseInt(selectedOption.dataset.height);
            const startX = parseInt(selectedOption.dataset.startX); const startY = parseInt(selectedOption.dataset.startY);
            const startAngle = parseFloat(selectedOption.dataset.startAngle);
            loadTrackImage(imageUrl, imgWidth, imgHeight, startX, startY, startAngle);
        } else if (trackImageSelector.options.length > 0) {
             trackImageSelector.selectedIndex = 0;
             trackImageSelector.dispatchEvent(new Event('change'));
        } else { /* ... no track ... */ 
            currentTrackImageData = undefined; 
            checkAndRenderInitialState(); 
        }
        updateInfoDisplayDefaults();
    }

    function updateInfoDisplayDefaults() {
        errorValSpan.textContent = "0.00"; pValSpan.textContent = "0.00"; iValSpan.textContent = "0.00";
        dValSpan.textContent = "0.00"; arduinoAjusteValSpan.textContent = "N/A";
        vLeftValSpan.textContent = "0"; vRightValSpan.textContent = "0";

        errorBar.style.height = '0%';
        pBar.style.height = '0%';
        iBar.style.height = '0%';
        dBar.style.height = '0%';
        adjPIDBar.style.height = '0%';
        vLeftBar.style.height = '0%';
        vRightBar.style.height = '0%';
    }

    function updateUIForSimulationState(isRunning) { /* ... same, check startButton logic carefully ... */ }
    function updateUIForSimulationState(isRunning) {
        const disableAllInputs = isRunning;
        startButton.disabled = isRunning || !currentTrackImageData || !currentTrackImage.complete;
        stopButton.disabled = !isRunning;
        resetButton.disabled = isRunning; 
        [timeStepInput, pixelsPerMeterDisplay, maxRobotSpeedMPSInput, motorResponseFactorInput,
         sensorNoiseProbInput, movementPerturbFactorInput, motorDeadbandPWMInput, lineThresholdInput,
         robotActualWidthInput, robotActualLengthInput, sideSensorSpreadInput, sensorForwardOffsetInput, sensorDiameterInput, 
         arduinoKpInput, arduinoKiInput, arduinoKdInput, arduinoVelBaseInput, arduinoVelRecInput, arduinoVelRevRecInput, arduinoIntegralMaxInput,
         trackImageSelector
        ].forEach(input => { input.disabled = disableAllInputs; });
        if (!isRunning && (!currentTrackImageData || !currentTrackImage.complete)) { startButton.disabled = true; }
    }

    startButton.addEventListener('click', startSimulation);
    stopButton.addEventListener('click', stopSimulation);
    resetButton.addEventListener('click', resetSimulation);
    trackImageSelector.addEventListener('change', (event) => { /* ... same ... */ });
    trackImageSelector.addEventListener('change', (event) => {
        const selectedOption = event.target.options[event.target.selectedIndex];
        const imageUrl = selectedOption.value;
        const imgWidth = parseInt(selectedOption.dataset.width); const imgHeight = parseInt(selectedOption.dataset.height);
        const startX = parseInt(selectedOption.dataset.startX); const startY = parseInt(selectedOption.dataset.startY);
        const startAngle = parseFloat(selectedOption.dataset.startAngle);
        startButton.disabled = true; 
        loadTrackImage(imageUrl, imgWidth, imgHeight, startX, startY, startAngle);
    });


    loadParameters(); 
    if (trackImageSelector.options.length > 0) {
        trackImageSelector.dispatchEvent(new Event('change'));
    } else { /* ... no tracks defined ... */ 
        displayCanvas.width = 800; displayCanvas.height = 600;
        currentTrackImageData = undefined; 
        checkAndRenderInitialState(); 
        updateUIForSimulationState(false);
        updateInfoDisplayDefaults();
        alert("No tracks defined.");
    }
});