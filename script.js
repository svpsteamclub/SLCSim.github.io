document.addEventListener('DOMContentLoaded', () => {
    const displayCanvas = document.getElementById('simulationCanvas'); // Canvas visible to user
    const displayCtx = displayCanvas.getContext('2d');

    // Create an off-screen canvas to hold the track image data for sampling
    const imageCanvas = document.createElement('canvas');
    const imageCtx = imageCanvas.getContext('2d', { willReadFrequently: true }); // Important for performance

    // --- DOM Elements for UI ---
    const trackImageSelector = document.getElementById('trackImageSelector');
    const timeStepInput = document.getElementById('timeStep');
    const pixelsPerMeterDisplay = document.getElementById('pixelsPerMeterDisplay'); // Display only
    const maxRobotSpeedMPSInput = document.getElementById('maxRobotSpeedMPS');
    const motorResponseFactorInput = document.getElementById('motorResponseFactor');
    const sensorNoiseProbInput = document.getElementById('sensorNoiseProb');        
    const movementPerturbFactorInput = document.getElementById('movementPerturbFactor'); 
    const motorDeadbandPWMInput = document.getElementById('motorDeadbandPWM');       
    const lineThresholdInput = document.getElementById('lineThreshold');
    
    // ... (Robot Geometry and Arduino Param Inputs - same getters as before) ...
    const robotActualWidthInput = document.getElementById('robotWidthInput_actual');
    const robotActualLengthInput = document.getElementById('robotLengthInput_actual');
    const sideSensorSpreadInput = document.getElementById('sideSensorSpreadInput');
    const sensorForwardOffsetInput = document.getElementById('sensorForwardOffsetInput');
    const arduinoKpInput = document.getElementById('arduino_kp');
    // ... (and the rest: Ki, Kd, VelBase, VelRec, VelRevRec, IntegralMax)
    const arduinoKiInput = document.getElementById('arduino_ki');
    const arduinoKdInput = document.getElementById('arduino_kd');
    const arduinoVelBaseInput = document.getElementById('arduino_velBase');
    const arduinoVelRecInput = document.getElementById('arduino_velRec');
    const arduinoVelRevRecInput = document.getElementById('arduino_velRevRec');
    const arduinoIntegralMaxInput = document.getElementById('arduino_integralMax');


    const startButton = document.getElementById('startButton'); // ... (buttons and info spans - same)
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

    const IMAGE_SCALE_FACTOR = 1000; // 350px on image = 0.35m => 1000 image_pixels per meter
    let pixelsPerMeter = IMAGE_SCALE_FACTOR; // This is now fixed by the image scale

    let currentTrackImage = new Image();
    let currentTrackImageData = null;
    let currentTrackWidth = 0;
    let currentTrackHeight = 0;
    
    // ... (Arduino PID state, motor state, visual params - same) ...
    let arduinoErrorPID = 0, arduinoErrorPrevioPID = 0, arduinoTerminoProporcional = 0;
    let arduinoTerminoIntegral = 0, arduinoTerminoDerivativo = 0, arduinoAjustePID = 0;
    let ultimaPosicionConocidaLinea = 0;
    let currentApplied_vR_mps = 0;
    let currentApplied_vL_mps = 0;
    const wheelWidth_px_vis = 8; 
    const wheelRadius_px_vis = 12; 
    const sensorRadiusPixels_vis = 4;


    let robot = { x_m: 0.1, y_m: 0.1, angle_rad: 0, trail: [] }; // Initial default

    // Track points editor removed. trackPoints_pixels is no longer used for line definition.

    let simulationRunning = false;
    let animationFrameId;
    let accumulator = 0;
    let lastFrameTime = performance.now();
    // isEditingTrack is removed.

    // --- Utility Functions --- (degreesToRadians, radiansToDegrees, clamp - same)
    function degreesToRadians(degrees) { return degrees * (Math.PI / 180); }
    function radiansToDegrees(radians) { return radians * (180 / Math.PI); }
    function clamp(value, min, max) { return Math.min(Math.max(value, min), max); }
    // getMousePos is no longer needed for track editing

    // --- Image Loading and Processing ---
    function loadTrackImage(imageUrl, imageWidth, imageHeight, startX_px, startY_px, startAngle_deg) {
        stopSimulation(); // Stop any current simulation
        currentTrackImage.onload = () => {
            currentTrackWidth = imageWidth; // Use dimensions from data attributes
            currentTrackHeight = imageHeight;

            // Resize both canvases
            displayCanvas.width = currentTrackWidth;
            displayCanvas.height = currentTrackHeight;
            imageCanvas.width = currentTrackWidth;
            imageCanvas.height = currentTrackHeight;

            imageCtx.drawImage(currentTrackImage, 0, 0, currentTrackWidth, currentTrackHeight);
            try {
                currentTrackImageData = imageCtx.getImageData(0, 0, currentTrackWidth, currentTrackHeight);
            } catch (e) {
                console.error("Error getting image data (CORS issue if loading from different domain locally?):", e);
                alert("Error loading track image data. Ensure images are served from the same origin or test on a local server.");
                currentTrackImageData = null; // Prevent further errors
                return;
            }
            
            console.log(`Track image ${imageUrl} loaded. Dimensions: ${currentTrackWidth}x${currentTrackHeight}`);
            
            // Set initial robot position based on data attributes for the selected track
            robot.x_m = startX_px / pixelsPerMeter;
            robot.y_m = startY_px / pixelsPerMeter;
            robot.angle_rad = degreesToRadians(startAngle_deg);
            robot.trail = [];

            resetArduinoPIDState(); // Reset PID for the new track
            render(null); // Render initial state
            updateUIForSimulationState(false);
        };
        currentTrackImage.onerror = () => {
            console.error(`Error loading track image: ${imageUrl}`);
            alert(`Could not load track image: ${imageUrl}. Make sure it's in the same folder as index.html.`);
            currentTrackImageData = null;
        };
        currentTrackImage.src = imageUrl;
    }
    
    // --- Robot and Sensor Drawing ---
    function drawRobot() { // Draws on displayCanvas
        displayCtx.save();
        displayCtx.translate(robot.x_m * pixelsPerMeter, robot.y_m * pixelsPerMeter);
        displayCtx.rotate(robot.angle_rad);
        const robotWheelbasePx = currentRobotWheelbase_m * pixelsPerMeter;
        const robotLengthPx = currentRobotLength_m * pixelsPerMeter;
        displayCtx.fillStyle = '#555555'; 
        const visualWheelDiameter = wheelRadius_px_vis * 2; 
        const visualWheelThickness = wheelWidth_px_vis;    
        displayCtx.fillRect(-visualWheelDiameter / 2, (-robotWheelbasePx / 2) - (visualWheelThickness / 2), visualWheelDiameter, visualWheelThickness);
        displayCtx.fillRect(-visualWheelDiameter / 2, (robotWheelbasePx / 2) - (visualWheelThickness / 2), visualWheelDiameter, visualWheelThickness);
        displayCtx.fillStyle = 'blue';
        displayCtx.fillRect(-robotLengthPx / 2, -robotWheelbasePx / 2, robotLengthPx, robotWheelbasePx);
        displayCtx.fillStyle = 'lightblue';
        displayCtx.beginPath();
        const indicatorTipX = robotLengthPx / 2 + 3; 
        const indicatorBaseX = robotLengthPx / 2 - Math.min(8, robotLengthPx * 0.15); 
        const indicatorBaseSpread = robotWheelbasePx / 3; 
        displayCtx.moveTo(indicatorTipX, 0); 
        displayCtx.lineTo(indicatorBaseX, -indicatorBaseSpread / 2); 
        displayCtx.lineTo(indicatorBaseX, indicatorBaseSpread / 2);  
        displayCtx.closePath(); displayCtx.fill();
        displayCtx.restore();
        if (robot.trail.length > 1) {
            displayCtx.beginPath(); displayCtx.strokeStyle = 'rgba(0, 0, 255, 0.3)'; displayCtx.lineWidth = 2;
            // Trail points are already in image pixels, but robot position is in meters.
            // For consistency, trail should store meter positions or be converted.
            // Let's store trail in meters and convert here.
            displayCtx.moveTo(robot.trail[0].x_m * pixelsPerMeter, robot.trail[0].y_m * pixelsPerMeter);
            for (let i = 1; i < robot.trail.length; i++) {
                displayCtx.lineTo(robot.trail[i].x_m * pixelsPerMeter, robot.trail[i].y_m * pixelsPerMeter);
            }
            displayCtx.stroke();
        }
    }

    function getSensorPositions_imagePx() { // Sensor positions in IMAGE pixel coordinates
        const sensorLineCenterX_m = robot.x_m + sensorForwardProtrusion_m * Math.cos(robot.angle_rad);
        const sensorLineCenterY_m = robot.y_m + sensorForwardProtrusion_m * Math.sin(robot.angle_rad);
        const perpendicularAngle = robot.angle_rad - Math.PI / 2;
        const lX_m = sensorLineCenterX_m + sensorSideSpread_m * Math.cos(perpendicularAngle);
        const lY_m = sensorLineCenterY_m + sensorSideSpread_m * Math.sin(perpendicularAngle);
        const cX_m = sensorLineCenterX_m; const cY_m = sensorLineCenterY_m;
        const rX_m = sensorLineCenterX_m - sensorSideSpread_m * Math.cos(perpendicularAngle);
        const rY_m = sensorLineCenterY_m - sensorSideSpread_m * Math.sin(perpendicularAngle);
        return { // Convert meter positions to image pixel coordinates
            left:   { x: Math.round(lX_m * pixelsPerMeter), y: Math.round(lY_m * pixelsPerMeter) },
            center: { x: Math.round(cX_m * pixelsPerMeter), y: Math.round(cY_m * pixelsPerMeter) },
            right:  { x: Math.round(rX_m * pixelsPerMeter), y: Math.round(rY_m * pixelsPerMeter) }
        };
    }

    function drawSensors(sensorStates) { // Draws on displayCanvas
        const positions_img_px = getSensorPositions_imagePx(); // Get sensor positions in image pixels
        // For drawing, these are the same pixel coordinates on the display canvas
        drawSensor(positions_img_px.left, sensorStates.left);
        drawSensor(positions_img_px.center, sensorStates.center);
        drawSensor(positions_img_px.right, sensorStates.right);
    }

    function drawSensor(pos_px, isOnLine) { // pos_px is in image/display pixel coords
        displayCtx.beginPath(); displayCtx.arc(pos_px.x, pos_px.y, sensorRadiusPixels_vis, 0, 2 * Math.PI);
        displayCtx.fillStyle = isOnLine ? 'red' : 'gray'; displayCtx.fill();
        displayCtx.strokeStyle = 'black'; displayCtx.lineWidth = 1; displayCtx.stroke();
    }

    // --- Line Detection from Image Data ---
    function isPixelOnLine(x_img_px, y_img_px) {
        if (!currentTrackImageData || x_img_px < 0 || x_img_px >= currentTrackWidth || y_img_px < 0 || y_img_px >= currentTrackHeight) {
            return false; // Out of bounds or no image data
        }
        const R_INDEX = (y_img_px * currentTrackWidth + x_img_px) * 4;
        // Simple grayscale: average of R,G,B. Or just use R if image is grayscale.
        // Assuming black line on white background.
        const r = currentTrackImageData.data[R_INDEX];
        const g = currentTrackImageData.data[R_INDEX + 1];
        const b = currentTrackImageData.data[R_INDEX + 2];
        const brightness = (r + g + b) / 3;
        return brightness < lineThreshold; // True if dark enough to be considered line
    }

    // --- Simulation Update (Fixed Time Step) ---
    function fixedUpdate(dt_s) {
        const sensorPositions_img_px = getSensorPositions_imagePx();
        let s_der_active = isPixelOnLine(sensorPositions_img_px.right.x, sensorPositions_img_px.right.y);
        let s_cen_active = isPixelOnLine(sensorPositions_img_px.center.x, sensorPositions_img_px.center.y);
        let s_izq_active = isPixelOnLine(sensorPositions_img_px.left.x, sensorPositions_img_px.left.y);

        // Apply Sensor Noise (same as before)
        if (simulationRunning && sensorNoiseProbability > 0) { /* ... apply noise ... */ }

        const S_DER = s_der_active ? 1 : 0; /* ... (rest of S_CEN, S_IZQ, lineaPerdida, error logic - same) ... */
        const S_CEN = s_cen_active ? 1 : 0; 
        const S_IZQ = s_izq_active ? 1 : 0;
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


        // ... (Motor control logic based on PID/recovery - same as before, setting velRawMotorA_target, velRawMotorB_target) ...
        let velRawMotorA_target = 0; let velRawMotorB_target = 0;
        let dirA_adelante = true; let dirB_adelante = true;

        if (lineaPerdida) { /* ... recovery ... */ } else { /* ... PID ... */ }
        // (Copy this whole block from previous script)
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


        // ... (Deadband, PWM clamping, m/s conversion, motor response lag - same as before, using finalVelRawMotorA/B, target_vR/L_mps, currentApplied_vR/L_mps) ...
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
        

        // ... (Kinematics, movement perturbations, robot position update, trail, boundary check - same as before) ...
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
        robot.trail.push({ x_m: robot.x_m, y_m: robot.y_m }); // Store meter positions in trail
        if (robot.trail.length > 500) robot.trail.shift();
        const boundaryMargin_m = Math.max(currentRobotWheelbase_m, currentRobotLength_m) / 2;
        if (robot.x_m < -boundaryMargin_m || robot.x_m * pixelsPerMeter > displayCanvas.width + boundaryMargin_m * pixelsPerMeter || 
            robot.y_m < -boundaryMargin_m || robot.y_m * pixelsPerMeter > displayCanvas.height + boundaryMargin_m * pixelsPerMeter) {
            stopSimulation();
        }

        return { left: s_izq_active, center: s_cen_active, right: s_der_active };
    }

    // --- Rendering ---
    function render(sensorStates) {
        displayCtx.clearRect(0, 0, displayCanvas.width, displayCanvas.height);
        // Draw the loaded track image onto the display canvas
        if (currentTrackImage.complete && currentTrackImage.src) {
            displayCtx.drawImage(currentTrackImage, 0, 0, displayCanvas.width, displayCanvas.height);
        } else { // Fallback if image not loaded yet
            displayCtx.fillStyle = '#ddd';
            displayCtx.fillRect(0,0, displayCanvas.width, displayCanvas.height);
            displayCtx.fillStyle = 'black';
            displayCtx.fillText("Loading track...", displayCanvas.width/2 - 30, displayCanvas.height/2);
        }

        if (simulationRunning || !isEditingTrack) { // isEditingTrack is always false now
            drawRobot(); // Will draw on displayCtx
            if (simulationRunning && sensorStates) drawSensors(sensorStates); // Will draw on displayCtx
        }
    }

    // --- Game Loop --- (Same)
    function gameLoop(currentTime) { /* ... */ }


    // --- Control Functions ---
    function loadParameters() {
        simTimeStep = parseFloat(timeStepInput.value);
        // pixelsPerMeter is now fixed by IMAGE_SCALE_FACTOR
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
        // ... (load other arduino and robot geometry params - same) ...
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
        if (!currentTrackImageData) {
            alert("Please select and load a track image first.");
            return;
        }
        loadParameters(); // Load all parameters
        simulationRunning = true;
        lastFrameTime = performance.now(); accumulator = 0;
        // resetArduinoPIDState(); // Robot position is set by loadTrackImage, PID reset there too
        // robot.trail should be reset when a new track is loaded.
        animationFrameId = requestAnimationFrame(gameLoop);
        updateUIForSimulationState(true);
    }
    
    function stopSimulation() { /* ... same ... */ }
    function resetArduinoPIDState() { /* ... same ... */ }
    // resetRobotPosition is now handled by loadTrackImage based on data attributes

    function resetSimulation() { // Now primarily reloads current track and parameters
        stopSimulation();
        loadParameters(); // Reload parameters from UI
        // Reload the currently selected track to reset robot position etc.
        const selectedOption = trackImageSelector.options[trackImageSelector.selectedIndex];
        if (selectedOption) {
            const imageUrl = selectedOption.value;
            const imgWidth = parseInt(selectedOption.dataset.width);
            const imgHeight = parseInt(selectedOption.dataset.height);
            const startX = parseInt(selectedOption.dataset.startX);
            const startY = parseInt(selectedOption.dataset.startY);
            const startAngle = parseFloat(selectedOption.dataset.startAngle);
            loadTrackImage(imageUrl, imgWidth, imgHeight, startX, startY, startAngle); // This also resets PID and renders
        }
        updateInfoDisplayDefaults();
        // updateUIForSimulationState(false); // loadTrackImage calls this via its own logic
    }

    function updateInfoDisplayDefaults() { /* ... same ... */ }

    // Track Editor Functions are REMOVED
    // handleCanvasClickForTrack, enterEditMode, clearTrack, finishEditing 

    function updateUIForSimulationState(isRunning) {
        const disableAllInputs = isRunning; // Only disable when running now
        startButton.disabled = isRunning || !currentTrackImageData;
        stopButton.disabled = !isRunning;
        resetButton.disabled = isRunning; // Can always reset if not running

        [timeStepInput, pixelsPerMeterDisplay, maxRobotSpeedMPSInput, motorResponseFactorInput,
         sensorNoiseProbInput, movementPerturbFactorInput, motorDeadbandPWMInput, lineThresholdInput,
         robotActualWidthInput, robotActualLengthInput, 
         sideSensorSpreadInput, sensorForwardOffsetInput,
         arduinoKpInput, arduinoKiInput, arduinoKdInput, arduinoVelBaseInput, 
         arduinoVelRecInput, arduinoVelRevRecInput, arduinoIntegralMaxInput,
         trackImageSelector // Also disable track selector when running
        ].forEach(input => { input.disabled = disableAllInputs; });
        
        // Editor buttons are removed from HTML
    }

    // --- Event Listeners ---
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
    
    // Initialize
    loadParameters(); 
    // Initial track load
    const initialOption = trackImageSelector.options[trackImageSelector.selectedIndex];
    if (initialOption) {
        loadTrackImage(
            initialOption.value, 
            parseInt(initialOption.dataset.width), 
            parseInt(initialOption.dataset.height),
            parseInt(initialOption.dataset.startX),
            parseInt(initialOption.dataset.startY),
            parseFloat(initialOption.dataset.startAngle)
        );
    } else {
        // Fallback if no tracks defined or selected
        displayCanvas.width = 800; displayCanvas.height = 600; // Default canvas size
        render(null); // Render empty state
        updateUIForSimulationState(false);
        updateInfoDisplayDefaults();
        alert("No tracks defined. Please add <option> tags to the #trackImageSelector in index.html.");
    }
});