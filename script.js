// --- Global Variables (Ensure these are at the top) ---
let robotImagesLoaded = false;
let watermarkImageLoaded = false;

// --- Track Configuration ---
// To add a new track:
// 1. Place the .png file in the same directory as index.html.
// 2. Add a new entry to this array with the track's details.
const AVAILABLE_TRACKS = [
    {
        displayName: "Pista 1 (1050x1050)",
        fileName: "track1_1050.png",
        width: 1050,
        height: 1050,
        startX: 500,
        startY: 875,
        startAngle: 0
    },
    {
        displayName: "Pista 2 (1400x1400)",
        fileName: "track2_1400.png",
        width: 1400,
        height: 1400,
        startX: 500,
        startY: 1225,
        startAngle: 0
    },
    {
        displayName: "Pista 3 (1750x1750)",
        fileName: "track3_1750.png",
        width: 1750,
        height: 1750,
        startX: 400,
        startY: 175,
        startAngle: 0
    },
	{
        displayName: "Pista de Pruebas I (940x1240)",
        fileName: "testTrack.png",
        width: 940,
        height: 1240,
        startX: 100,
        startY: 150,
        startAngle: 0
    },
	{
        displayName: "Pista en Meson (1750x1040)",
        fileName: "PistaMeson.png",
        width: 1750,
        height: 1040,
        startX: 500,
        startY: 350,
        startAngle: 0
    },
];


const WHEEL_LENGTH_M = 0.07;
const WHEEL_WIDTH_M = 0.03;
const MAX_BAR_HEIGHT_PX = 50;
let currentMaxValError = 2.5;
let currentMaxValPTerm = 150;
let currentMaxValITerm = 50;
let currentMaxValDTerm = 150;
let currentMaxValAdjPID = 255;
const MAX_VAL_PWM_BAR = 255;

// --- Lap Timing Variables ---
let initialLapState = { x_m: 0, y_m: 0, angle_rad: 0 };
let lapStartTime_sim_s = 0;
let totalSimulationTime_s = 0;
let lapTimes = [];
let bestLapTime_s = Infinity;
let hasLeftStartZone = false;
let lapCounter = 0;
let robot_x_m_previous_tick = 0;
let robot_y_m_previous_tick = 0;


// --- Global DOM Element Variables ---
let displayCanvas, displayCtx, imageCanvas, imageCtx;
let trackImageSelector, timeStepInput, pixelsPerMeterDisplay, maxRobotSpeedMPSInput, motorResponseFactorInput,
    sensorNoiseProbInput, movementPerturbFactorInput, motorDeadbandPWMInput, lineThresholdInput,
    robotActualWidthInput, robotActualLengthInput, sideSensorSpreadInput, sensorForwardOffsetInput, sensorDiameterInput,
    arduinoKpInput, arduinoKiInput, arduinoKdInput, arduinoVelBaseInput,
    arduinoIntegralMaxInput, startButton, stopButton, resetButton, setStartPositionButton;
let errorValSpan, pValSpan, iValSpan, dValSpan, arduinoAjusteValSpan, vLeftValSpan, vRightValSpan;
let errorBar, pBar, iBar, dBar, adjPIDBar, vLeftBar, vRightBar;
let currentLapTimeValSpan, bestLapTimeValSpan, lapTimesTableBody;
let customTrackInput, clearCustomTrackButton;


// --- Global Simulation State Variables ---
let simTimeStep, maxPhysicalSpeed_mps, currentMotorResponseFactor;
let sensorNoiseProbability, movementPerturbationFactor, motorDeadbandPWMValue, lineThreshold;
let currentRobotWheelbase_m, currentRobotLength_m, sensorSideSpread_m, sensorForwardProtrusion_m, currentSensorDiameter_m;
const IMAGE_SCALE_FACTOR = 1000;
let pixelsPerMeter = IMAGE_SCALE_FACTOR;
const robotBodyImage = new Image();
const robotWheelImage = new Image();
const watermarkImage = new Image();
let currentTrackImage = new Image();
let currentTrackImageData = null;
let currentTrackWidth_imgPx = 0;
let currentTrackHeight_imgPx = 0;

let arduinoErrorPID = 0, arduinoErrorPrevioPID = 0, arduinoTerminoProporcional = 0;
let arduinoAcumuladorIntegral = 0, arduinoTerminoIntegral = 0, arduinoTerminoDerivativo = 0, arduinoAjustePID = 0;
let ultimaPosicionConocidaLinea = 0;
let arduinoKp, arduinoKi, arduinoKd, arduinoVelBase, arduinoIntegralMax;


let currentApplied_vR_mps = 0;
let currentApplied_vL_mps = 0;
let robot = { x_m: 0.1, y_m: 0.1, angle_rad: 0, centerTrail: [], leftWheelTrail: [], rightWheelTrail: [] };
let simulationRunning = false;
let animationFrameId;
let accumulator = 0;
let lastFrameTime = performance.now();

let isSettingStartPosition = false;
let startPositionClickPoint_canvasPx = { x: null, y: null };
let currentMousePosition_canvasPx = { x: null, y: null };

// --- Custom Track Variables ---
let isCustomTrackActive = false;
let customTrackStart = { x_m: 0.1, y_m: 0.1, angle_rad: 0 }; // Stores user-defined start for the custom track
let customTrackImageFilename = "";


// --- Global Functions ---
function degreesToRadians(degrees) { return degrees * (Math.PI / 180); }
function radiansToDegrees(radians) { return radians * (180 / Math.PI); }
function clamp(value, min, max) { return Math.min(Math.max(value, min), max); }

function getMousePos(canvas, evt) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    return {
        x: (evt.clientX - rect.left) * scaleX,
        y: (evt.clientY - rect.top) * scaleY
    };
}

function checkAndRenderInitialState() {
    if (!displayCanvas) {
        return;
    }
    if (robotImagesLoaded && watermarkImageLoaded) {
        render(null);
        if (!simulationRunning && typeof updateUIForSimulationState === 'function') {
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

function loadTrackImage(imageUrl, imageWidthPx, imageHeightPx, startX_imgPx, startY_imgPx, startAngle_deg) {
    if (isSettingStartPosition) {
        toggleSetStartPositionMode(); // Exit mode if active
    }
    stopSimulation();

    // Explicitly clear custom track state if we are loading a predefined track
    isCustomTrackActive = false;
    customTrackImageFilename = "";
    if (customTrackInput) customTrackInput.value = null; // Clear file input
    // UI update for buttons/selector handled by updateUIForSimulationState called later or by event handlers

    currentTrackImageData = null;
    currentTrackImage = new Image(); // Create a new Image object for the predefined track
    currentTrackImage.onload = () => {
        if (!displayCanvas) {
            console.error("loadTrackImage.onload: displayCanvas not initialized!");
            return;
        }
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
            console.error("Error getting image data for predefined track:", e);
            currentTrackImageData = null;
            checkAndRenderInitialState();
            updateUIForSimulationState(false); // Ensure UI reflects failed load
            return;
        }
        console.log("Predefined track image loaded and data processed:", imageUrl);
        robot.x_m = startX_imgPx / pixelsPerMeter;
        robot.y_m = startY_imgPx / pixelsPerMeter;
        robot.angle_rad = degreesToRadians(startAngle_deg);

        initializeLapTiming();

        robot.centerTrail = []; robot.leftWheelTrail = []; robot.rightWheelTrail = [];
        resetArduinoPIDState();
        updateInfoDisplayDefaults();
        checkAndRenderInitialState();
        updateUIForSimulationState(false);
        if(startButton && currentTrackImageData) startButton.disabled = false; else if (startButton) startButton.disabled = true;
    };
    currentTrackImage.onerror = () => {
        console.error(`Error loading track image: ${imageUrl}`);
        currentTrackImageData = null;
        initializeLapTiming();
        checkAndRenderInitialState();
        if(startButton) startButton.disabled = true;
        updateUIForSimulationState(false);
    };
    currentTrackImage.src = imageUrl;
}


function processAndLoadCustomTrack(loadedImageObject, fileName) {
    if (isSettingStartPosition) {
        toggleSetStartPositionMode();
    }
    stopSimulation();

    isCustomTrackActive = true;
    customTrackImageFilename = fileName;
    currentTrackImage = loadedImageObject; // This is already the loaded Image object

    currentTrackWidth_imgPx = currentTrackImage.naturalWidth;
    currentTrackHeight_imgPx = currentTrackImage.naturalHeight;

    console.log(`Custom track "${fileName}" loaded. Dimensions: ${currentTrackWidth_imgPx}x${currentTrackHeight_imgPx}`);

    displayCanvas.width = currentTrackWidth_imgPx;
    displayCanvas.height = currentTrackHeight_imgPx;
    imageCanvas.width = currentTrackWidth_imgPx;
    imageCanvas.height = currentTrackHeight_imgPx;

    imageCtx.drawImage(currentTrackImage, 0, 0, currentTrackWidth_imgPx, currentTrackHeight_imgPx);
    try {
        currentTrackImageData = imageCtx.getImageData(0, 0, currentTrackWidth_imgPx, currentTrackHeight_imgPx);
    } catch (e) {
        console.error("Error getting image data for custom track:", e);
        alert(`Error processing custom track image: ${e.message}. Track might not render correctly.`);
        currentTrackImageData = null;
        isCustomTrackActive = false; // Revert state
        checkAndRenderInitialState();
        updateUIForSimulationState(false);
        return;
    }

    // Set a default initial position (e.g., center), user MUST set it properly
    robot.x_m = (currentTrackWidth_imgPx / 2) / pixelsPerMeter;
    robot.y_m = (currentTrackHeight_imgPx / 2) / pixelsPerMeter;
    robot.angle_rad = 0; // Default angle

    // Store this as the initial custom start, to be updated by user
    customTrackStart = { x_m: robot.x_m, y_m: robot.y_m, angle_rad: robot.angle_rad };

    initializeLapTiming();
    robot.centerTrail = []; robot.leftWheelTrail = []; robot.rightWheelTrail = [];
    resetArduinoPIDState();
    updateInfoDisplayDefaults();
    checkAndRenderInitialState(); // Render the newly loaded custom track
    updateUIForSimulationState(false); // Update button states etc.

    // Prompt user to set start position
    alert(`Pista "${fileName}" cargada. Por favor, establece la posición y dirección inicial del robot usando el botón "Posición y Dirección Inicial" o haciendo clic y arrastrando directamente sobre la pista.`);
    if (!isSettingStartPosition) {
        toggleSetStartPositionMode(); // Automatically enter mode to set start position
    }
    if (startButton) startButton.disabled = true; // Keep start disabled until position is confirmed
}


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
    const sensorRadiusPx = (currentSensorDiameter_m / 2) * pixelsPerMeter;
    displayCtx.beginPath();
    displayCtx.arc(pos_px.x, pos_px.y, Math.max(1, sensorRadiusPx), 0, 2 * Math.PI);
    displayCtx.fillStyle = isOnLine ? 'green' : 'gray';
    displayCtx.fill();
    displayCtx.strokeStyle = 'black';
    displayCtx.lineWidth = 1;
    displayCtx.stroke();
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
function updateBar(barElement, value, maxValue, valueTextElement) {
    let absValue = 0;
    let heightPercentage = 0;
    if (valueTextElement && isNaN(parseFloat(valueTextElement.textContent))) {
        heightPercentage = 0;
    } else if (typeof value === 'number' && !isNaN(value)) {
        absValue = Math.abs(value);
        if (maxValue > 0.00001) {
            heightPercentage = Math.min(100, (absValue / maxValue) * 100);
        }
    }
    if (isNaN(heightPercentage)) { heightPercentage = 0;}
    if (barElement) barElement.style.height = `${heightPercentage}%`; else console.warn("updateBar: barElement is null for value:", value);
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

    const S_DER = s_der_active ? 1 : 0;
    const S_CEN = s_cen_active ? 1 : 0;
    const S_IZQ = s_izq_active ? 1 : 0;

    if (S_DER === 0 && S_CEN === 0 && S_IZQ === 0) {
        if (ultimaPosicionConocidaLinea === 1) {
            arduinoErrorPID = -currentMaxValError;
        } else if (ultimaPosicionConocidaLinea === 2) {
            arduinoErrorPID = currentMaxValError;
        } else {
             if (Math.abs(arduinoErrorPrevioPID) > 0.1) {
                 arduinoErrorPID = arduinoErrorPrevioPID;
            } else {
                 arduinoErrorPID = currentMaxValError;
            }
        }
    } else if (S_DER === 0 && S_CEN === 1 && S_IZQ === 0) {
        arduinoErrorPID = 0.0;
        ultimaPosicionConocidaLinea = 0;
    } else if (S_DER === 0 && S_CEN === 1 && S_IZQ === 1) {
        arduinoErrorPID = -0.5;
        ultimaPosicionConocidaLinea = 1;
    } else if (S_DER === 0 && S_CEN === 0 && S_IZQ === 1) {
        arduinoErrorPID = -2.0;
        ultimaPosicionConocidaLinea = 1;
    } else if (S_DER === 1 && S_CEN === 1 && S_IZQ === 0) {
        arduinoErrorPID = 0.5;
        ultimaPosicionConocidaLinea = 2;
    } else if (S_DER === 1 && S_CEN === 0 && S_IZQ === 0) {
        arduinoErrorPID = 2.0;
        ultimaPosicionConocidaLinea = 2;
    } else if (S_DER === 1 && S_CEN === 1 && S_IZQ === 1) {
        arduinoErrorPID = 0.0;
        ultimaPosicionConocidaLinea = 0;
    } else if (S_DER === 1 && S_CEN === 0 && S_IZQ === 1) {
        arduinoErrorPID = arduinoErrorPrevioPID;
    }
   
    arduinoTerminoProporcional = arduinoKp * arduinoErrorPID;

    arduinoAcumuladorIntegral += arduinoErrorPID * dt_s;
    if (arduinoIntegralMax > 0) {
        arduinoAcumuladorIntegral = clamp(arduinoAcumuladorIntegral, -arduinoIntegralMax, arduinoIntegralMax);
    } else {
        arduinoAcumuladorIntegral = 0;
    }
    arduinoTerminoIntegral = arduinoKi * arduinoAcumuladorIntegral;
   
    if (dt_s > 0.0001) {
        arduinoTerminoDerivativo = arduinoKd * (arduinoErrorPID - arduinoErrorPrevioPID) / dt_s;
    } else {
        arduinoTerminoDerivativo = 0;
    }
    arduinoErrorPrevioPID = arduinoErrorPID;

    arduinoAjustePID = arduinoTerminoProporcional + arduinoTerminoIntegral + arduinoTerminoDerivativo;

    let rawSpeedMotorDerecho = arduinoVelBase - arduinoAjustePID;
    let rawSpeedMotorIzquierdo = arduinoVelBase + arduinoAjustePID;

    let dirDerechoForward = rawSpeedMotorDerecho >= 0;
    let dirIzquierdoForward = rawSpeedMotorIzquierdo >= 0;

    let pwmMotorDerecho = Math.abs(rawSpeedMotorDerecho);
    let pwmMotorIzquierdo = Math.abs(rawSpeedMotorIzquierdo);

    if (pwmMotorDerecho < motorDeadbandPWMValue) {
        pwmMotorDerecho = 0;
    }
    if (pwmMotorIzquierdo < motorDeadbandPWMValue) {
        pwmMotorIzquierdo = 0;
    }

    pwmMotorDerecho = clamp(pwmMotorDerecho, 0, 255);
    pwmMotorIzquierdo = clamp(pwmMotorIzquierdo, 0, 255);
   
    errorValSpan.textContent = arduinoErrorPID.toFixed(2);
    updateBar(errorBar, arduinoErrorPID, currentMaxValError, errorValSpan);
    pValSpan.textContent = arduinoTerminoProporcional.toFixed(2);
    updateBar(pBar, arduinoTerminoProporcional, currentMaxValPTerm, pValSpan);
    iValSpan.textContent = arduinoTerminoIntegral.toFixed(2);
    updateBar(iBar, arduinoTerminoIntegral, currentMaxValITerm, iValSpan);
    dValSpan.textContent = arduinoTerminoDerivativo.toFixed(2);
    updateBar(dBar, arduinoTerminoDerivativo, currentMaxValDTerm, dValSpan);
    arduinoAjusteValSpan.textContent = arduinoAjustePID.toFixed(2);
    updateBar(adjPIDBar, arduinoAjustePID, currentMaxValAdjPID, arduinoAjusteValSpan);
   
    vLeftValSpan.textContent = Math.round(pwmMotorIzquierdo);
    vRightValSpan.textContent = Math.round(pwmMotorDerecho);
    updateBar(vLeftBar, pwmMotorIzquierdo, MAX_VAL_PWM_BAR, vLeftValSpan);
    updateBar(vRightBar, pwmMotorDerecho, MAX_VAL_PWM_BAR, vRightValSpan);

    let target_vR_mps = (dirDerechoForward ? 1 : -1) * (pwmMotorDerecho / 255.0) * maxPhysicalSpeed_mps;
    let target_vL_mps = (dirIzquierdoForward ? 1 : -1) * (pwmMotorIzquierdo / 255.0) * maxPhysicalSpeed_mps;
   
    if (pwmMotorDerecho === 0) target_vR_mps = 0;
    if (pwmMotorIzquierdo === 0) target_vL_mps = 0;
       
    currentApplied_vR_mps += (target_vR_mps - currentApplied_vR_mps) * currentMotorResponseFactor;
    currentApplied_vL_mps += (target_vL_mps - currentApplied_vL_mps) * currentMotorResponseFactor;

    const L_m = currentRobotWheelbase_m;
    let d_theta_rad = 0;
    if (L_m > 0.001) {
        d_theta_rad = -(currentApplied_vR_mps - currentApplied_vL_mps) / L_m * dt_s;
    }
    let linear_displacement_m = (currentApplied_vR_mps + currentApplied_vL_mps) / 2.0 * dt_s;

    if (simulationRunning && movementPerturbationFactor > 0) {
        const perturbR = (Math.random() * 2 - 1) * movementPerturbationFactor;
        const perturbL = (Math.random() * 2 - 1) * movementPerturbationFactor;
        linear_displacement_m *= (1 + perturbR);
        d_theta_rad *= (1 + perturbL);
    }

    robot.angle_rad += d_theta_rad;
    robot.angle_rad = Math.atan2(Math.sin(robot.angle_rad), Math.cos(robot.angle_rad));
    robot.x_m += linear_displacement_m * Math.cos(robot.angle_rad);
    robot.y_m += linear_displacement_m * Math.sin(robot.angle_rad);

    robot.centerTrail.push({ x_m: robot.x_m, y_m: robot.y_m }); if (robot.centerTrail.length > 500) robot.centerTrail.shift();
    const halfWheelbase_m = currentRobotWheelbase_m / 2; const sinAngle = Math.sin(robot.angle_rad); const cosAngle = Math.cos(robot.angle_rad);
    const x_lw_m = robot.x_m + halfWheelbase_m * sinAngle; const y_lw_m = robot.y_m - halfWheelbase_m * cosAngle; robot.leftWheelTrail.push({ x_m: x_lw_m, y_m: y_lw_m }); if (robot.leftWheelTrail.length > 500) robot.leftWheelTrail.shift();
    const x_rw_m = robot.x_m - halfWheelbase_m * sinAngle; const y_rw_m = robot.y_m + halfWheelbase_m * cosAngle; robot.rightWheelTrail.push({ x_m: x_rw_m, y_m: y_rw_m }); if (robot.rightWheelTrail.length > 500) robot.rightWheelTrail.shift();

    if (simulationRunning) {
        totalSimulationTime_s += dt_s;
        let currentLapDisplayTime_s = totalSimulationTime_s - lapStartTime_sim_s;
        updateCurrentLapTimeDisplay(currentLapDisplayTime_s);

        const START_LINE_TOLERANCE_LATERAL_M = currentRobotWheelbase_m * 0.75;
        const START_ZONE_EXIT_DISTANCE_M = currentRobotLength_m * 1.25;

        if (!hasLeftStartZone) {
            const distFromStartPointSq = Math.pow(robot.x_m - initialLapState.x_m, 2) + Math.pow(robot.y_m - initialLapState.y_m, 2);
            if (distFromStartPointSq > Math.pow(START_ZONE_EXIT_DISTANCE_M, 2)) {
                hasLeftStartZone = true;
            }
        }

        if (hasLeftStartZone) {
            const P0_x = initialLapState.x_m;
            const P0_y = initialLapState.y_m;
            const startAngle = initialLapState.angle_rad;

            const D_x = Math.cos(startAngle);
            const D_y = Math.sin(startAngle);

            const V_prev_x = robot_x_m_previous_tick - P0_x;
            const V_prev_y = robot_y_m_previous_tick - P0_y;
            const V_curr_x = robot.x_m - P0_x;
            const V_curr_y = robot.y_m - P0_y;

            const proj_prev = V_prev_x * D_x + V_prev_y * D_y;
            const proj_curr = V_curr_x * D_x + V_curr_y * D_y;

            const epsilon = 1e-3;
            if (proj_prev <= epsilon && proj_curr > epsilon) {
                const N_x = -D_y;
                const N_y = D_x;
                const lateral_offset = Math.abs(V_curr_x * N_x + V_curr_y * N_y);

                if (lateral_offset < START_LINE_TOLERANCE_LATERAL_M) {
                    lapCounter++;
                    const completedLapTime = totalSimulationTime_s - lapStartTime_sim_s;

                    lapTimes.unshift(completedLapTime);
                    if (lapTimes.length > 5) {
                        lapTimes.pop();
                    }

                    if (completedLapTime < bestLapTime_s) {
                        bestLapTime_s = completedLapTime;
                        updateBestLapTimeDisplay();
                    }

                    lapStartTime_sim_s = totalSimulationTime_s;
                    hasLeftStartZone = false;

                    updateLapTimesDisplayTable();
                }
            }
        }
        robot_x_m_previous_tick = robot.x_m;
        robot_y_m_previous_tick = robot.y_m;
    }

    const boundaryMargin_m = Math.max(currentRobotWheelbase_m, currentRobotLength_m) / 2; if (robot.x_m < -boundaryMargin_m || robot.x_m * pixelsPerMeter > displayCanvas.width + boundaryMargin_m * pixelsPerMeter || robot.y_m < -boundaryMargin_m || robot.y_m * pixelsPerMeter > displayCanvas.height + boundaryMargin_m * pixelsPerMeter) { stopSimulation(); }
    return { left: s_izq_active, center: s_cen_active, right: s_der_active };
}

function render(sensorStates) {
    displayCtx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);
    if (currentTrackImageData && currentTrackImage.complete && currentTrackImage.naturalWidth > 0) {
        displayCtx.drawImage(currentTrackImage, 0, 0, displayCanvas.width, displayCanvas.height);
        if (watermarkImageLoaded && watermarkImage.complete && watermarkImage.naturalWidth > 0) {
            const watermarkAspectRatio = watermarkImage.naturalWidth / watermarkImage.naturalHeight;
            let watermarkWidth = displayCanvas.width * 0.6; let watermarkHeight = watermarkWidth / watermarkAspectRatio;
            if (watermarkHeight > displayCanvas.height * 0.6) { watermarkHeight = displayCanvas.height * 0.6; watermarkWidth = watermarkHeight * watermarkAspectRatio; }
            if (watermarkWidth > displayCanvas.width * 0.6) { watermarkWidth = displayCanvas.width * 0.6; watermarkHeight = watermarkWidth / watermarkAspectRatio; }
            const watermarkX = (displayCanvas.width - watermarkWidth) / 2; const watermarkY = (displayCanvas.height - watermarkHeight) / 2;
            displayCtx.save(); displayCtx.globalAlpha = 0.2; displayCtx.drawImage(watermarkImage, watermarkX, watermarkY, watermarkWidth, watermarkHeight); displayCtx.restore();
        }
    } else {
        displayCtx.fillStyle = '#eee'; displayCtx.fillRect(0, 0, displayCanvas.width, displayCanvas.height);
        displayCtx.fillStyle = 'black'; displayCtx.textAlign = 'center'; displayCtx.font = "16px Arial";
        let message = "Loading assets...";
        if (isCustomTrackActive && !currentTrackImageData && currentTrackImage.src && currentTrackImage.src.startsWith('data:image')) {
            message = `Cargando pista personalizada: ${customTrackImageFilename}...`;
        } else if (trackImageSelector && trackImageSelector.value && !currentTrackImageData && currentTrackImage.src.endsWith(trackImageSelector.value) && !currentTrackImage.complete) {
             message = `Cargando pista: ${trackImageSelector.options[trackImageSelector.selectedIndex].text}...`;
        }
        else if (trackImageSelector && (!trackImageSelector.value || trackImageSelector.value === "") && AVAILABLE_TRACKS.length === 0 && !isCustomTrackActive) {
            message = "No hay pistas configuradas. Agregue pistas a AVAILABLE_TRACKS en script.js o cargue una personalizada.";
        }
        else if (isCustomTrackActive && currentTrackImageData === null && currentTrackImage.src) {
             message = `Error al cargar pista personalizada: ${customTrackImageFilename}. Revise la consola.`;
        }
        else if (currentTrackImageData === null && currentTrackImage.src) { // For predefined tracks
             message = `Error al cargar pista: ${currentTrackImage.src.split('/').pop()}. Revise la consola.`;
        }
        else if (currentTrackImageData === undefined) { // No track loaded at all (predefined or custom)
            message = "Seleccione una pista predefinida o cargue una pista personalizada para comenzar.";
        }
        displayCtx.fillText(message, displayCanvas.width / 2, displayCanvas.height / 2);
    }

    if (isSettingStartPosition && startPositionClickPoint_canvasPx.x !== null && currentMousePosition_canvasPx.x !== null) {
        if (startPositionClickPoint_canvasPx.x !== currentMousePosition_canvasPx.x || startPositionClickPoint_canvasPx.y !== currentMousePosition_canvasPx.y) {
            displayCtx.save();
            displayCtx.beginPath();
            displayCtx.moveTo(startPositionClickPoint_canvasPx.x, startPositionClickPoint_canvasPx.y);
            displayCtx.lineTo(currentMousePosition_canvasPx.x, currentMousePosition_canvasPx.y);
            displayCtx.strokeStyle = 'rgba(255, 0, 0, 0.7)';
            displayCtx.lineWidth = 2;
            displayCtx.setLineDash([5, 5]);
            displayCtx.stroke();
            displayCtx.setLineDash([]);
            displayCtx.restore();
        }
    }
    if (isSettingStartPosition) {
        displayCtx.fillStyle = "rgba(0,0,0,0.8)";
        displayCtx.font = "bold 14px Arial";
        displayCtx.textAlign = "center";
        const instructionText = startPositionClickPoint_canvasPx.x === null ?
            (isCustomTrackActive ? "Pista personalizada cargada. Haz clic para fijar la posición inicial." : "Haz clic en la pista para fijar la posición inicial del robot.") :
            "Arrastra para definir el ángulo. Suelta el ratón para confirmar.";
        displayCtx.fillText(instructionText, displayCanvas.width / 2, 25);
    }


    if (currentTrackImageData !== undefined && currentTrackImageData !== null && robotImagesLoaded) {
         if(currentRobotLength_m > 0 && currentRobotWheelbase_m > 0) {
            drawRobot();
            if (simulationRunning && sensorStates) drawSensors(sensorStates);
        }
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
    pixelsPerMeterDisplay.value = IMAGE_SCALE_FACTOR.toFixed(0); pixelsPerMeter = IMAGE_SCALE_FACTOR;
    maxPhysicalSpeed_mps = parseFloat(maxRobotSpeedMPSInput.value);
    currentMotorResponseFactor = parseFloat(motorResponseFactorInput.value); currentMotorResponseFactor = clamp(currentMotorResponseFactor, 0.01, 1.0);
    sensorNoiseProbability = parseFloat(sensorNoiseProbInput.value); sensorNoiseProbability = clamp(sensorNoiseProbability, 0, 1);
    movementPerturbationFactor = parseFloat(movementPerturbFactorInput.value); movementPerturbationFactor = clamp(movementPerturbationFactor, 0, 0.5);
    motorDeadbandPWMValue = parseFloat(motorDeadbandPWMInput.value); motorDeadbandPWMValue = clamp(motorDeadbandPWMValue, 0, 255);
    lineThreshold = parseInt(lineThresholdInput.value); lineThreshold = clamp(lineThreshold, 0, 255);
    currentRobotWheelbase_m = parseFloat(robotActualWidthInput.value); currentRobotLength_m = parseFloat(robotActualLengthInput.value);
    sensorSideSpread_m = parseFloat(sideSensorSpreadInput.value); sensorForwardProtrusion_m = parseFloat(sensorForwardOffsetInput.value);
    currentSensorDiameter_m = parseFloat(sensorDiameterInput.value);
   
    arduinoKp = parseFloat(arduinoKpInput.value);
    arduinoKi = parseFloat(arduinoKiInput.value);
    arduinoKd = parseFloat(arduinoKdInput.value);
    arduinoVelBase = parseFloat(arduinoVelBaseInput.value);
    arduinoIntegralMax = parseFloat(arduinoIntegralMaxInput.value);

    currentMaxValAdjPID = arduinoVelBase + 255;
}
function startSimulation() {
    if (simulationRunning) return;
    if (isSettingStartPosition) {
        // If trying to start while still in "set start position" mode,
        // (e.g. for a custom track where it's mandatory first)
        // we should prevent starting or auto-confirm the current temp position.
        // For now, let's assume 'setStartPositionButton' handles exiting this mode.
        // Or, if start is clicked, we can finalize the current position.
        // Simpler: 'setStartPositionButton' makes 'startButton' available.
        alert("Por favor, confirme la posición inicial antes de simular.");
        return;
    }
    if (!currentTrackImageData) {
        alert("Por favor, seleccione o cargue una pista y establezca la posición inicial.");
        return;
    }


    loadParameters();
    simulationRunning = true;
    lastFrameTime = performance.now();
    accumulator = 0;
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
    arduinoAcumuladorIntegral = 0; arduinoTerminoIntegral = 0; arduinoTerminoDerivativo = 0; arduinoAjustePID = 0;
    ultimaPosicionConocidaLinea = 0;
    currentApplied_vL_mps = 0; currentApplied_vR_mps = 0;
}
function resetSimulation() {
    if (isSettingStartPosition) {
        toggleSetStartPositionMode(); // Exit set start position mode first
    }
    stopSimulation();
    loadParameters();

    if (isCustomTrackActive && currentTrackImage.src && currentTrackImage.complete) {
        // Reload custom track with its user-defined start
        robot.x_m = customTrackStart.x_m;
        robot.y_m = customTrackStart.y_m;
        robot.angle_rad = customTrackStart.angle_rad;

        // Ensure canvas and image data are set up for the custom track
        // (currentTrackImage should still hold the loaded custom Image object)
        displayCanvas.width = currentTrackWidth_imgPx; // Should be custom track's width
        displayCanvas.height = currentTrackHeight_imgPx; // Should be custom track's height
        imageCanvas.width = currentTrackWidth_imgPx;
        imageCanvas.height = currentTrackHeight_imgPx;
        imageCtx.drawImage(currentTrackImage, 0, 0, currentTrackWidth_imgPx, currentTrackHeight_imgPx);
        try {
            currentTrackImageData = imageCtx.getImageData(0, 0, currentTrackWidth_imgPx, currentTrackHeight_imgPx);
        } catch (e) {
            console.error("Error getting image data for custom track on reset:", e);
            currentTrackImageData = null; // Mark as failed
            isCustomTrackActive = false; // Cannot proceed with this custom track
        }

        initializeLapTiming();
        robot.centerTrail = []; robot.leftWheelTrail = []; robot.rightWheelTrail = [];
        resetArduinoPIDState();
        updateInfoDisplayDefaults();
        checkAndRenderInitialState(); // This will render based on currentTrackImageData
        updateUIForSimulationState(false);
        if(startButton && currentTrackImageData) startButton.disabled = false; else if(startButton) startButton.disabled = true;

    } else {
        // Logic for pre-defined tracks
        isCustomTrackActive = false; // Ensure it's false

        const selectedIdx = trackImageSelector.selectedIndex;
        if (selectedIdx >= 0 && trackImageSelector.options[selectedIdx] &&
            trackImageSelector.options[selectedIdx].value !== "" && AVAILABLE_TRACKS.length > 0) {
            const selectedOption = trackImageSelector.options[selectedIdx];
            const imageUrl = selectedOption.value;
            const imgWidth = parseInt(selectedOption.dataset.width);
            const imgHeight = parseInt(selectedOption.dataset.height);
            const startX = parseInt(selectedOption.dataset.startX); // Use potentially modified start
            const startY = parseInt(selectedOption.dataset.startY);
            const startAngle = parseFloat(selectedOption.dataset.startAngle);

            if (startButton) startButton.disabled = true;
            loadTrackImage(imageUrl, imgWidth, imgHeight, startX, startY, startAngle);
        } else if (AVAILABLE_TRACKS.length > 0 && trackImageSelector.options.length > 0 && trackImageSelector.options[0].value !== "") {
            trackImageSelector.selectedIndex = 0;
            trackImageSelector.dispatchEvent(new Event('change'));
        } else {
            // No tracks (predefined or custom) are effectively loaded
            currentTrackImage = new Image(); // Clear current track image
            currentTrackImageData = undefined;
            robot.x_m = 0.1; robot.y_m = 0.1; robot.angle_rad = 0;
            robot.centerTrail = []; robot.leftWheelTrail = []; robot.rightWheelTrail = [];
            resetArduinoPIDState();
            initializeLapTiming();
            if (displayCanvas && displayCtx) {
                displayCanvas.width = displayCanvas.width || 800;
                displayCanvas.height = displayCanvas.height || 600;
                displayCtx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);
            }
            checkAndRenderInitialState();
            updateInfoDisplayDefaults();
            updateUIForSimulationState(false);
            if(startButton) startButton.disabled = true;
            console.warn("ResetSimulation: No valid track to load.");
        }
    }
}


function updateInfoDisplayDefaults() {
    errorValSpan.textContent = "0.00"; pValSpan.textContent = "0.00"; iValSpan.textContent = "0.00";
    dValSpan.textContent = "0.00"; arduinoAjusteValSpan.textContent = "N/A";
    vLeftValSpan.textContent = "0"; vRightValSpan.textContent = "0";
    if(errorBar) errorBar.style.height = '0%'; if(pBar) pBar.style.height = '0%'; if(iBar) iBar.style.height = '0%';
    if(dBar) dBar.style.height = '0%'; if(adjPIDBar) adjPIDBar.style.height = '0%';
    if(vLeftBar) vLeftBar.style.height = '0%'; if(vRightBar) vRightBar.style.height = '0%';
}

function updateUIForSimulationState(isRunning) {
    const generalDisable = isRunning || isSettingStartPosition;
    const noTrackEffectivelyLoaded = (!currentTrackImageData && currentTrackImageData !== null) || // Explicitly null means error, undefined means not set
                                   (currentTrackImage.src && !currentTrackImage.complete) || // Image source set but not loaded
                                   currentTrackImageData === null; // Explicitly errored

    if (startButton) {
        startButton.disabled = isRunning || isSettingStartPosition || noTrackEffectivelyLoaded;
    }
    if (stopButton) stopButton.disabled = !isRunning;
    if (resetButton) resetButton.disabled = isRunning || isSettingStartPosition; // Allow reset even if track error, to try again
    if (setStartPositionButton) {
         setStartPositionButton.disabled = isRunning || noTrackEffectivelyLoaded;
    }

    [timeStepInput, maxRobotSpeedMPSInput, motorResponseFactorInput,
     sensorNoiseProbInput, movementPerturbFactorInput, motorDeadbandPWMInput, lineThresholdInput,
     robotActualWidthInput, robotActualLengthInput, sideSensorSpreadInput, sensorForwardOffsetInput, sensorDiameterInput,
     arduinoKpInput, arduinoKiInput, arduinoKdInput, arduinoVelBaseInput, arduinoIntegralMaxInput
    ].forEach(input => { if (input) input.disabled = generalDisable; });

    if (trackImageSelector) {
        trackImageSelector.disabled = generalDisable || isCustomTrackActive || (AVAILABLE_TRACKS.length === 0);
    }
    if (customTrackInput) {
        customTrackInput.disabled = generalDisable || isCustomTrackActive;
    }
    if (clearCustomTrackButton) {
        clearCustomTrackButton.disabled = generalDisable || !isCustomTrackActive;
        clearCustomTrackButton.style.display = isCustomTrackActive ? 'inline-block' : 'none';
    }
}


function toggleSetStartPositionMode() {
    isSettingStartPosition = !isSettingStartPosition;
    if (isSettingStartPosition) {
        if (simulationRunning) { // Should not happen if UI is correct
            isSettingStartPosition = false; return;
        }
        if (!currentTrackImageData && currentTrackImageData !== null) { // No track loaded or error
            alert("Por favor, cargue una pista primero.");
            isSettingStartPosition = false; return;
        }
        setStartPositionButton.textContent = "Cancelar Ajuste";
        setStartPositionButton.style.backgroundColor = "#ffc107"; // Yellow
        displayCanvas.style.cursor = 'crosshair';
        if(startButton) startButton.disabled = true; // Disable start while actively setting position
    } else {
        setStartPositionButton.textContent = "Posición y Dirección Inicial";
        setStartPositionButton.style.backgroundColor = "#6c757d"; // Default grey
        displayCanvas.style.cursor = 'default';
        startPositionClickPoint_canvasPx = { x: null, y: null };
        currentMousePosition_canvasPx = { x: null, y: null };
        document.removeEventListener('mousemove', handleCanvasMouseMove);
        document.removeEventListener('mouseup', handleCanvasMouseUp);
        // Re-enable start button if a track is loaded and ready
        if (currentTrackImageData && startButton) {
            startButton.disabled = false;
        }
    }
    updateUIForSimulationState(simulationRunning); // Reflects new isSettingStartPosition state
    render(null); // Re-render to show/hide instruction text
}


function handleCanvasMouseDown(event) {
    if (!isSettingStartPosition || simulationRunning) return;

    const pos = getMousePos(displayCanvas, event);
    startPositionClickPoint_canvasPx = { x: pos.x, y: pos.y };
    currentMousePosition_canvasPx = { x: pos.x, y: pos.y };

    robot.x_m = pos.x / pixelsPerMeter;
    robot.y_m = pos.y / pixelsPerMeter;
    robot.angle_rad = 0; // Temporarily set angle to 0, will be updated by drag

    // Add listeners to canvas instead of document
    displayCanvas.addEventListener('mousemove', handleCanvasMouseMove);
    displayCanvas.addEventListener('mouseup', handleCanvasMouseUp);
    displayCanvas.addEventListener('mouseleave', handleCanvasMouseUp);

    render(null); // Re-render to show the start of the angle drag indicator
}

function handleCanvasMouseMove(event) {
    if (!isSettingStartPosition || !startPositionClickPoint_canvasPx.x) return;

    const pos = getMousePos(displayCanvas, event);
    currentMousePosition_canvasPx = { x: pos.x, y: pos.y };

    const dx = currentMousePosition_canvasPx.x - startPositionClickPoint_canvasPx.x;
    const dy = currentMousePosition_canvasPx.y - startPositionClickPoint_canvasPx.y;

    if (Math.sqrt(dx * dx + dy * dy) > 5) { // Only update angle if mouse moved a bit
        // Calculate angle relative to positive y-axis (up) instead of x-axis
        robot.angle_rad = Math.atan2(dx, -dy);
    }
    render(null); // Re-render to show angle indicator updating
}

function handleCanvasMouseUp(event) {
    // Remove listeners from canvas
    displayCanvas.removeEventListener('mousemove', handleCanvasMouseMove);
    displayCanvas.removeEventListener('mouseup', handleCanvasMouseUp);
    displayCanvas.removeEventListener('mouseleave', handleCanvasMouseUp);

    if (!isSettingStartPosition) return;

    console.log(`Nueva posición inicial fijada: X=${robot.x_m.toFixed(3)}m, Y=${robot.y_m.toFixed(3)}m, Angulo=${radiansToDegrees(robot.angle_rad).toFixed(1)}deg`);

    // Now finalize the robot state based on this new position
    robot.centerTrail = []; robot.leftWheelTrail = []; robot.rightWheelTrail = [];
    resetArduinoPIDState();
    updateInfoDisplayDefaults();

    if (isCustomTrackActive) {
        customTrackStart.x_m = robot.x_m;
        customTrackStart.y_m = robot.y_m;
        customTrackStart.angle_rad = robot.angle_rad;
        console.log("Posición inicial de pista personalizada actualizada.");
    } else {
        // For predefined tracks, update its entry in AVAILABLE_TRACKS's data attributes for session persistence via reset
        const selectedOption = trackImageSelector.options[trackImageSelector.selectedIndex];
        if (selectedOption && selectedOption.dataset) {
            selectedOption.dataset.startX = Math.round(robot.x_m * pixelsPerMeter);
            selectedOption.dataset.startY = Math.round(robot.y_m * pixelsPerMeter);
            selectedOption.dataset.startAngle = radiansToDegrees(robot.angle_rad).toFixed(1);
        }
    }

    initializeLapTiming(); // Initialize lap timing with the new confirmed start position

    if (isSettingStartPosition) { // Should be true here
        toggleSetStartPositionMode(); // Exits mode, re-enables start button if track is ready
    }
}


// --- Custom Track Handling Functions ---
function handleCustomTrackUpload(event) {
    const file = event.target.files[0];
    if (!file) {
        return;
    }

    if (file.type !== "image/png") {
        alert("Por favor, seleccione un archivo de imagen PNG.");
        customTrackInput.value = null; // Reset file input
        return;
    }

    const reader = new FileReader();
    reader.onload = function(e) {
        const img = new Image();
        img.onload = function() {
            processAndLoadCustomTrack(img, file.name);
        };
        img.onerror = function() {
            alert("Error al cargar la imagen de la pista personalizada.");
            customTrackInput.value = null;
        };
        img.src = e.target.result;
    };
    reader.onerror = function() {
        alert("Error al leer el archivo de la pista.");
        customTrackInput.value = null;
    };
    reader.readAsDataURL(file);
}

function clearCustomTrack() {
    if (isSettingStartPosition) {
        toggleSetStartPositionMode(); // Exit mode first
    }
    stopSimulation();

    isCustomTrackActive = false;
    customTrackImageFilename = "";
    currentTrackImage = new Image(); // Reset to a blank image
    currentTrackImageData = undefined; // No track data
    if (customTrackInput) customTrackInput.value = null; // Clear the file input

    // Re-enable predefined track selector and load the selected (or first) one
    if (trackImageSelector) {
        trackImageSelector.disabled = (AVAILABLE_TRACKS.length === 0);
        if (AVAILABLE_TRACKS.length > 0) {
            const selectedIdx = trackImageSelector.selectedIndex >= 0 ? trackImageSelector.selectedIndex : 0;
            if (trackImageSelector.options.length > selectedIdx) {
                 trackImageSelector.selectedIndex = selectedIdx; // Ensure a valid selection
                 trackImageSelector.dispatchEvent(new Event('change')); // Trigger load of predefined track
            } else { // Fallback if selector is somehow empty but shouldn't be
                loadDefaultEmptyState();
            }
        } else { // No predefined tracks exist
            loadDefaultEmptyState();
        }
    } else { // Should not happen if HTML is correct
        loadDefaultEmptyState();
    }
    updateUIForSimulationState(false);
}

function loadDefaultEmptyState() {
    console.warn("No predefined tracks available to load after clearing custom track.");
    displayCanvas.width = displayCanvas.width || 800; // Keep current or default size
    displayCanvas.height = displayCanvas.height || 600;
    displayCtx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);
    robot.x_m = 0.1; robot.y_m = 0.1; robot.angle_rad = 0;
    initializeLapTiming();
    resetArduinoPIDState();
    updateInfoDisplayDefaults();
    checkAndRenderInitialState(); // Will show "Select a track..."
    if (startButton) startButton.disabled = true;
}


document.addEventListener('DOMContentLoaded', () => {
    displayCanvas = document.getElementById('simulationCanvas');
    displayCtx = displayCanvas.getContext('2d');
    imageCanvas = document.createElement('canvas');
    imageCtx = imageCanvas.getContext('2d', { willReadFrequently: true });
    trackImageSelector = document.getElementById('trackImageSelector');
    timeStepInput = document.getElementById('timeStep');
    pixelsPerMeterDisplay = document.getElementById('pixelsPerMeterDisplay');
    maxRobotSpeedMPSInput = document.getElementById('maxRobotSpeedMPS');
    motorResponseFactorInput = document.getElementById('motorResponseFactor');
    sensorNoiseProbInput = document.getElementById('sensorNoiseProb');
    movementPerturbFactorInput = document.getElementById('movementPerturbFactor');
    motorDeadbandPWMInput = document.getElementById('motorDeadbandPWM');
    lineThresholdInput = document.getElementById('lineThreshold');
    robotActualWidthInput = document.getElementById('robotWidthInput_actual');
    robotActualLengthInput = document.getElementById('robotLengthInput_actual');
    sideSensorSpreadInput = document.getElementById('sideSensorSpreadInput');
    sensorForwardOffsetInput = document.getElementById('sensorForwardOffsetInput');
    sensorDiameterInput = document.getElementById('sensorDiameterInput');
   
    arduinoKpInput = document.getElementById('arduino_kp');
    arduinoKiInput = document.getElementById('arduino_ki');
    arduinoKdInput = document.getElementById('arduino_kd');
    arduinoVelBaseInput = document.getElementById('arduino_velBase');
    arduinoIntegralMaxInput = document.getElementById('arduino_integralMax');
   
    startButton = document.getElementById('startButton');
    stopButton = document.getElementById('stopButton');
    resetButton = document.getElementById('resetButton');
    setStartPositionButton = document.getElementById('setStartPositionButton');
   
    errorValSpan = document.getElementById('errorVal');
    pValSpan = document.getElementById('pVal');
    iValSpan = document.getElementById('iVal');
    dValSpan = document.getElementById('dVal');
    arduinoAjusteValSpan = document.getElementById('arduinoAjusteVal');
    vLeftValSpan = document.getElementById('vLeftVal');
    vRightValSpan = document.getElementById('vRightVal');
   
    errorBar = document.getElementById('errorBar');
    pBar = document.getElementById('pBar');
    iBar = document.getElementById('iBar');
    dBar = document.getElementById('dBar');
    adjPIDBar = document.getElementById('adjPIDBar');
    vLeftBar = document.getElementById('vLeftBar');
    vRightBar = document.getElementById('vRightBar');

    currentLapTimeValSpan = document.getElementById('currentLapTimeVal');
    bestLapTimeValSpan = document.getElementById('bestLapTimeVal');
    lapTimesTableBody = document.querySelector('#lapTimesTable tbody');

    customTrackInput = document.getElementById('customTrackInput');
    clearCustomTrackButton = document.getElementById('clearCustomTrackButton');


    function populateTrackSelector() {
        trackImageSelector.innerHTML = '';

        if (AVAILABLE_TRACKS.length === 0) {
            const option = document.createElement('option');
            option.value = "";
            option.textContent = "No hay pistas predefinidas";
            trackImageSelector.appendChild(option);
            // trackImageSelector.disabled = true; // Disabling handled by updateUI
            // Don't call checkAndRenderInitialState here, let the main flow handle it
            return;
        }

        // trackImageSelector.disabled = false; // Handled by updateUI
        AVAILABLE_TRACKS.forEach((track, index) => {
            const option = document.createElement('option');
            option.value = track.fileName;
            option.textContent = track.displayName;
            option.dataset.width = track.width;
            option.dataset.height = track.height;
            option.dataset.startX = track.startX;
            option.dataset.startY = track.startY;
            option.dataset.startAngle = track.startAngle;

            if (index === 0) {
                option.selected = true;
            }
            trackImageSelector.appendChild(option);
        });
    }

    populateTrackSelector();
    loadParameters();
    loadRobotGraphics();
    loadWatermarkGraphic();

    startButton.addEventListener('click', startSimulation);
    stopButton.addEventListener('click', stopSimulation);
    resetButton.addEventListener('click', resetSimulation);
    setStartPositionButton.addEventListener('click', toggleSetStartPositionMode);
    displayCanvas.addEventListener('mousedown', handleCanvasMouseDown);

    customTrackInput.addEventListener('change', handleCustomTrackUpload);
    clearCustomTrackButton.addEventListener('click', clearCustomTrack);


    trackImageSelector.addEventListener('change', (event) => {
        // If a custom track was active, selecting a predefined track will implicitly clear it.
        // The loadTrackImage function handles resetting isCustomTrackActive flag.

        const selectedOption = event.target.options[event.target.selectedIndex];
        if (selectedOption && selectedOption.value && selectedOption.value !== "") {
            const imageUrl = selectedOption.value;
            const imgWidth = parseInt(selectedOption.dataset.width);
            const imgHeight = parseInt(selectedOption.dataset.height);
            const startX = parseInt(selectedOption.dataset.startX);
            const startY = parseInt(selectedOption.dataset.startY);
            const startAngle = parseFloat(selectedOption.dataset.startAngle);
            if (startButton) startButton.disabled = true;
            loadTrackImage(imageUrl, imgWidth, imgHeight, startX, startY, startAngle);
        } else {
            // Handle "No tracks configured" or empty selection
            isCustomTrackActive = false; // Ensure custom track is not considered active
            currentTrackImage = new Image();
            currentTrackImageData = undefined;
            initializeLapTiming();
            checkAndRenderInitialState(); // Will show appropriate message
            updateUIForSimulationState(false);
            updateInfoDisplayDefaults();
            if(startButton) startButton.disabled = true;
        }
    });

    const foldableTitles = document.querySelectorAll('.foldable-title');
    foldableTitles.forEach(title => {
        title.addEventListener('click', () => {
            const content = title.nextElementSibling;
            const indicator = title.querySelector('.fold-indicator');

            if (content && content.classList.contains('foldable-content')) {
                if (content.style.display === 'none' || content.style.display === '') {
                    content.style.display = 'block';
                    if (indicator) indicator.textContent = '[-]';
                } else {
                    content.style.display = 'none';
                    if (indicator) indicator.textContent = '[+]';
                }
            }
        });
    });

    // Initial setup
    if (AVAILABLE_TRACKS.length > 0) {
        trackImageSelector.dispatchEvent(new Event('change')); // Load the first pre-defined track
    } else {
        // No predefined tracks, set up a blank state
        currentTrackImageData = undefined; // Means no track is loaded
        initializeLapTiming();
        checkAndRenderInitialState(); // Will show "Select a track..." or "No tracks..."
        updateInfoDisplayDefaults();
        if(startButton) startButton.disabled = true;
        console.log("No predefined tracks configured in AVAILABLE_TRACKS array.");
    }
    updateUIForSimulationState(false); // Final UI state check
});


function initializeLapTiming() {
    initialLapState = { x_m: robot.x_m, y_m: robot.y_m, angle_rad: robot.angle_rad };
    lapStartTime_sim_s = 0;
    totalSimulationTime_s = 0;
    lapTimes = [];
    bestLapTime_s = Infinity;
    hasLeftStartZone = false;
    lapCounter = 0;

    robot_x_m_previous_tick = robot.x_m;
    robot_y_m_previous_tick = robot.y_m;

    updateLapTimesDisplayTable();
    updateBestLapTimeDisplay();
    updateCurrentLapTimeDisplay(0);
}

function updateCurrentLapTimeDisplay(time_s) {
    if (currentLapTimeValSpan) currentLapTimeValSpan.textContent = time_s.toFixed(3);
}

function updateBestLapTimeDisplay() {
    if (bestLapTimeValSpan) {
        bestLapTimeValSpan.textContent = bestLapTime_s === Infinity ? "N/A" : bestLapTime_s.toFixed(3) + " s";
    }
}

function updateLapTimesDisplayTable() {
    if (!lapTimesTableBody) return;
    lapTimesTableBody.innerHTML = '';

    const displayLaps = lapTimes.slice(0, 5);
    for (let i = 0; i < displayLaps.length; i++) {
        const row = lapTimesTableBody.insertRow();
        const cellLapNum = row.insertCell();
        const cellLapTime = row.insertCell();

        cellLapNum.textContent = lapCounter - i;
        cellLapTime.textContent = displayLaps[i].toFixed(3);
    }
}