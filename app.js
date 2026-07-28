/* ==========================================================================
   AI Waste Segregation Robotic Arm — Engineering Simulation Engine
   Grounded in detect_script.py (Pi) & newprogramwaste.ino (Arduino UNO)
   ========================================================================== */

document.addEventListener("DOMContentLoaded", () => {
  // ====== Material Properties (Technical Palettes & Bins) ======
  const MATERIALS = {
    cardboard: {
      name: "Cardboard",
      code: 1,
      color: "var(--mat-cardboard)",
      binAngle: 0,
      confidence: 0.94,
      box: { xmin: 0.22, ymin: 0.25, xmax: 0.78, ymax: 0.75 }
    },
    glass: {
      name: "Glass",
      code: 2,
      color: "var(--mat-glass)",
      binAngle: 45,
      confidence: 0.88,
      box: { xmin: 0.35, ymin: 0.20, xmax: 0.65, ymax: 0.80 }
    },
    metal: {
      name: "Metal",
      code: 3,
      color: "var(--mat-metal)",
      binAngle: 90,
      confidence: 0.91,
      box: { xmin: 0.30, ymin: 0.28, xmax: 0.70, ymax: 0.72 }
    },
    paper: {
      name: "Paper",
      code: 4,
      color: "var(--mat-paper)",
      binAngle: 135,
      confidence: 0.86,
      box: { xmin: 0.25, ymin: 0.30, xmax: 0.75, ymax: 0.70 }
    },
    plastic: {
      name: "Plastic",
      code: 5,
      color: "var(--mat-plastic)",
      binAngle: 180,
      confidence: 0.95,
      box: { xmin: 0.32, ymin: 0.22, xmax: 0.68, ymax: 0.78 }
    }
  };

  const MATERIAL_KEYS = Object.keys(MATERIALS);

  // ====== 8 Pipeline Stages ======
  const STAGES = [
    {
      id: "FRAME_CAPTURE",
      number: 1,
      title: "STAGE 1/8: FRAME CAPTURE & RESIZING",
      explanation: "The camera mounted on the arm gripper takes an uncompressed video frame of the intake tray. The image matrix is resized to 300x300 pixels to match the input tensor shape of the MobileNet SSD TFLite model.",
      filename: "detect_script.py (Lines 64-69)",
      code: `# ====== Read Video Frame & Resize ======
ret, frame = cap.read()
input_image = cv2.resize(frame, (width, height))
input_data = np.expand_dims(input_image, axis=0).astype(np.uint8)`
    },
    {
      id: "TFLITE_INFERENCE",
      number: 2,
      title: "STAGE 2/8: TFLITE MODEL INFERENCE",
      explanation: "The 300x300 tensor is evaluated by the TFLite Interpreter. The Single Shot MultiBox Detector (SSD) outputs bounding box coordinates [ymin, xmin, ymax, xmax], class ID indices, and confidence scores.",
      filename: "detect_script.py (Lines 71-76)",
      code: `# ====== Run TFLite Interpreter ======
interpreter.set_tensor(input_details[0]['index'], input_data)
interpreter.invoke()

boxes = interpreter.get_tensor(output_details[0]['index'])[0]
classes = interpreter.get_tensor(output_details[1]['index'])[0].astype(int)
scores = interpreter.get_tensor(output_details[2]['index'])[0]`
    },
    {
      id: "DEBOUNCE_CHECK",
      number: 3,
      title: "STAGE 3/8: 5-FRAME DEBOUNCING FILTER",
      explanation: "DESIGN DECISION: To avoid false triggers from light reflections or transient misreadings, a waste class must achieve 5 CONSECUTIVE confident detections (> 0.5 confidence score) before activating the robotic arm.",
      filename: "detect_script.py (Lines 94-110)",
      code: `# ====== Debouncing Validation ======
if confidence > CONFIDENCE_THRESHOLD:
    object_counts[label] += 1
    if object_counts[label] >= DETECTION_THRESHOLD: # 5 consecutive
        send_command_to_arduino(material_code)
        object_counts = {k: 0 for k in object_counts}`
    },
    {
      id: "SERIAL_SEND",
      number: 4,
      title: "STAGE 4/8: SERIAL PACKET TRANSMISSION",
      explanation: "Upon debouncing validation, the Raspberry Pi transmits a serial command packet over COM7 @ 9600 baud containing the material classification code (1=Cardboard, 2=Glass, 3=Metal, 4=Paper, 5=Plastic).",
      filename: "detect_script.py (Lines 42-48)",
      code: `# ====== Send Command to Arduino ======
def send_command_to_arduino(code):
    if arduino:
        print(f"[SEND COMMAND] -> {code}")
        arduino.write(f"{code}\\n".encode())
        time.sleep(0.1)`
    },
    {
      id: "ARDUINO_LED_FLASH",
      number: 5,
      title: "STAGE 5/8: ARDUINO HARDWARE ACKNOWLEDGMENT",
      explanation: "The Arduino parses the incoming packet via Serial.parseInt(). It immediately flashes the built-in Pin 13 LED N times to visually acknowledge command receipt to operators before engaging motors.",
      filename: "newprogramwaste.ino (Lines 36-43, 79)",
      code: `// ====== Visual LED Ack ======
void flash(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500);
  }
}`
    },
    {
      id: "ARM_PICKUP",
      number: 6,
      title: "STAGE 6/8: ROBOTIC ARM PICKUP INTERPOLATION",
      explanation: "Arduino calls pickUp(). Joint servos (Base, Shoulder, Elbow, Wrist, Gripper) sweep degree-by-degree (30ms per step) to reach down into the intake tray and close the gripper claw around the item.",
      filename: "newprogramwaste.ino (Lines 100-114, 117-202)",
      code: `// ====== Gradual Servo Interpolation ======
void sweep(Servo servo, int oldPos, int newPos, int servoSpeed) {
  for (oldPos; oldPos <= newPos; oldPos += 1) {
    servo.write(oldPos);
    delay(servoSpeed); // Degree-by-degree sweep
  }
}`
    },
    {
      id: "ARM_TRANSIT_DROPOFF",
      number: 7,
      title: "STAGE 7/8: BIN TRANSIT & DROPOFF",
      explanation: "Arduino calls dropOff(). The arm lifts the item, rotates the Base servo to the fixed output bin angle (Cardboard 0°, Glass 45°, Metal 90°, Paper 135°, Plastic 180°), and opens the gripper to deposit the item.",
      filename: "newprogramwaste.ino (Lines 205-280)",
      code: `// ====== Output Bin Rotation ======
void dropOff() {
  if (material == 1) { // Cardboard
    sweep(base, basePos, 0, 30);   // Rotate base to 0 deg
    sweep(hand, handPos, 160, 30); // Release gripper
  }
}`
    },
    {
      id: "ARM_HOMESTATE",
      number: 8,
      title: "STAGE 8/8: RETURN TO SCAN POSE & READY REPLY",
      explanation: "Arduino executes homeState(), resetting servos back to resting posture facing down into the intake tray. It transmits 'Done Moving' over serial to signal readiness for the next scanning cycle.",
      filename: "newprogramwaste.ino (Lines 87-88, 283-294)",
      code: `// ====== Reset Pose & Completion Signal ======
homeState(); // Facing down into intake tray
Serial.println("Done Moving");
Serial.flush();`
    }
  ];

  // ====== State Engine Variables ======
  let currentStageIndex = 0;
  let isPlaying = false;
  let speedMultiplier = 1.0;
  let manualOverride = "auto";
  let currentMaterialKey = "cardboard";
  let currentItemIndex = 0;
  let consecutiveCounts = { cardboard: 0, glass: 0, metal: 0, paper: 0, plastic: 0 };
  let simulationTimer = null;
  let currentTheme = localStorage.getItem("theme") || "dark";

  // Servo Positions (Angles in degrees)
  let servoAngles = {
    base: 90,
    shoulder: 65,
    elbow: 110,
    wrist1: 90,
    wrist2: 60,
    hand: 90 // 90=Open, 180=Closed
  };

  // Item position tracking in arena
  let itemInArena = {
    x: 580,
    y: 240,
    attachedToGripper: false,
    droppedInBin: false
  };

  // ====== DOM Selectors ======
  const btnPlayPause = document.getElementById("btnPlayPause");
  const playSvg = document.getElementById("playSvg");
  const playText = document.getElementById("playText");
  const btnPrev = document.getElementById("btnPrev");
  const btnNext = document.getElementById("btnNext");
  const btnRestart = document.getElementById("btnRestart");
  const speedSlider = document.getElementById("speedSlider");
  const speedVal = document.getElementById("speedVal");
  const itemSelect = document.getElementById("itemSelect");
  const codeToggle = document.getElementById("codeToggle");
  const themeToggle = document.getElementById("themeToggle");
  const themeLabelText = document.getElementById("themeLabelText");
  const btnClearSerial = document.getElementById("btnClearSerial");

  const cameraSvg = document.getElementById("cameraSvg");
  const detectionBox = document.getElementById("detectionBox");
  const boxLabel = document.getElementById("boxLabel");
  const debounceStatusText = document.getElementById("debounceStatusText");

  const armSvg = document.getElementById("armSvg");
  const ledIndicator = document.getElementById("ledIndicator");
  const serialLog = document.getElementById("serialLog");

  const stageTitle = document.getElementById("stageTitle");
  const stageBadge = document.getElementById("stageBadge");
  const stageExplanation = document.getElementById("stageExplanation");
  const codeFilename = document.getElementById("codeFilename");
  const codeSnippetText = document.getElementById("codeSnippetText");
  const codeSnippetContainer = document.getElementById("codeSnippetContainer");

  const angleBase = document.getElementById("angleBase");
  const angleShoulder = document.getElementById("angleShoulder");
  const angleElbow = document.getElementById("angleElbow");
  const angleWrist1 = document.getElementById("angleWrist1");
  const angleWrist2 = document.getElementById("angleWrist2");
  const angleHand = document.getElementById("angleHand");

  // ====== Initializer ======
  function init() {
    applyTheme(currentTheme);
    setupEventListeners();
    renderCameraSvg();
    renderArmSvg();
    updateUI();
  }

  function setupEventListeners() {
    btnPlayPause.addEventListener("click", togglePlayPause);
    btnPrev.addEventListener("click", prevStep);
    btnNext.addEventListener("click", nextStep);
    btnRestart.addEventListener("click", restartCycle);

    speedSlider.addEventListener("input", (e) => {
      speedMultiplier = parseFloat(e.target.value);
      speedVal.textContent = `${speedMultiplier.toFixed(1)}x`;
    });

    itemSelect.addEventListener("change", (e) => {
      manualOverride = e.target.value;
      restartCycle();
    });

    codeToggle.addEventListener("change", (e) => {
      if (e.target.checked) {
        codeSnippetContainer.classList.remove("hidden");
      } else {
        codeSnippetContainer.classList.add("hidden");
      }
    });

    themeToggle.addEventListener("change", (e) => {
      currentTheme = e.target.checked ? "light" : "dark";
      applyTheme(currentTheme);
    });

    btnClearSerial.addEventListener("click", () => {
      serialLog.innerHTML = `<div class="log-line info">[INIT] Serial log cleared.</div>`;
    });
  }

  function applyTheme(theme) {
    document.documentElement.setAttribute("data-theme", theme);
    themeToggle.checked = (theme === "light");
    themeLabelText.textContent = (theme === "light") ? "DARK MODE" : "LIGHT MODE";
    localStorage.setItem("theme", theme);
    renderCameraSvg();
    renderArmSvg();
  }

  // ====== Playback Control ======
  function togglePlayPause() {
    isPlaying = !isPlaying;
    if (isPlaying) {
      playSvg.innerHTML = `<rect x="6" y="4" width="4" height="16"></rect><rect x="14" y="4" width="4" height="16"></rect>`;
      playText.textContent = "PAUSE";
      runLoop();
    } else {
      playSvg.innerHTML = `<polygon points="5,3 19,12 5,21"></polygon>`;
      playText.textContent = "PLAY";
      clearTimeout(simulationTimer);
    }
  }

  function runLoop() {
    if (!isPlaying) return;

    const stageDuration = Math.max(400, 1400 / speedMultiplier);
    simulationTimer = setTimeout(() => {
      if (currentStageIndex < STAGES.length - 1) {
        currentStageIndex++;
      } else {
        pickNextItem();
        currentStageIndex = 0;
      }
      executeCurrentStage();
      runLoop();
    }, stageDuration);
  }

  function nextStep() {
    if (isPlaying) togglePlayPause();
    if (currentStageIndex < STAGES.length - 1) {
      currentStageIndex++;
    } else {
      pickNextItem();
      currentStageIndex = 0;
    }
    executeCurrentStage();
  }

  function prevStep() {
    if (isPlaying) togglePlayPause();
    if (currentStageIndex > 0) {
      currentStageIndex--;
    } else {
      currentStageIndex = 0;
    }
    executeCurrentStage();
  }

  function restartCycle() {
    if (isPlaying) togglePlayPause();
    currentStageIndex = 0;
    consecutiveCounts = { cardboard: 0, glass: 0, metal: 0, paper: 0, plastic: 0 };
    pickNextItem();
    executeCurrentStage();
  }

  function pickNextItem() {
    if (manualOverride !== "auto") {
      currentMaterialKey = manualOverride;
    } else {
      currentItemIndex = (currentItemIndex + 1) % MATERIAL_KEYS.length;
      currentMaterialKey = MATERIAL_KEYS[currentItemIndex];
    }
    itemInArena.x = 580;
    itemInArena.y = 240;
    itemInArena.attachedToGripper = false;
    itemInArena.droppedInBin = false;
  }

  // ====== Execute Pipeline Stage ======
  function executeCurrentStage() {
    const stage = STAGES[currentStageIndex];
    const mat = MATERIALS[currentMaterialKey];

    if (stage.id !== "ARDUINO_LED_FLASH") {
      ledIndicator.classList.remove("led-on");
      ledIndicator.textContent = "PIN 13 LED: INACTIVE";
    }

    switch (stage.id) {
      case "FRAME_CAPTURE":
        detectionBox.classList.add("hidden");
        break;

      case "TFLITE_INFERENCE":
        detectionBox.classList.remove("hidden");
        const b = mat.box;
        detectionBox.style.left = `${b.xmin * 100}%`;
        detectionBox.style.top = `${b.ymin * 100}%`;
        detectionBox.style.width = `${(b.xmax - b.xmin) * 100}%`;
        detectionBox.style.height = `${(b.ymax - b.ymin) * 100}%`;
        boxLabel.textContent = `${mat.name.toUpperCase()} [CONF: ${mat.confidence.toFixed(2)}]`;
        break;

      case "DEBOUNCE_CHECK":
        consecutiveCounts[currentMaterialKey] = 5;
        debounceStatusText.textContent = `5 / 5 CONFIRMED`;
        break;

      case "SERIAL_SEND":
        logSerial(`[PI → ARDUINO] PACKET SENT: Code=${mat.code} (${mat.name.toUpperCase()}), Zone=1, Angle=${mat.binAngle}°`, "tx");
        break;

      case "ARDUINO_LED_FLASH":
        ledIndicator.classList.add("led-on");
        ledIndicator.textContent = `PIN 13 LED: FLASHING (${mat.code}X)`;
        logSerial(`[ARDUINO] Executed flash(${mat.code}) on Pin 13 LED.`, "rx");
        break;

      case "ARM_PICKUP":
        servoAngles = { base: 25, shoulder: 45, elbow: 180, wrist1: 90, wrist2: 35, hand: 180 };
        itemInArena.attachedToGripper = true;
        break;

      case "ARM_TRANSIT_DROPOFF":
        servoAngles = { base: mat.binAngle, shoulder: 90, elbow: 0, wrist1: 90, wrist2: 90, hand: 90 };
        itemInArena.attachedToGripper = false;
        itemInArena.droppedInBin = true;
        logSerial(`[ARDUINO] Executed dropOff() -> Rotated Base to ${mat.binAngle}° & Released Gripper.`, "rx");
        break;

      case "ARM_HOMESTATE":
        servoAngles = { base: 90, shoulder: 65, elbow: 110, wrist1: 90, wrist2: 60, hand: 90 };
        logSerial(`[ARDUINO → PI] TX: "Done Moving"`, "highlight");
        break;
    }

    updateUI();
  }

  // ====== Serial Log Utility ======
  function logSerial(msg, type = "info") {
    const timeStr = new Date().toLocaleTimeString();
    const line = document.createElement("div");
    line.className = `log-line ${type}`;
    line.textContent = `[${timeStr}] ${msg}`;
    serialLog.appendChild(line);
    serialLog.scrollTop = serialLog.scrollHeight;
  }

  // ====== Update UI Labels & Graphics ======
  function updateUI() {
    const stage = STAGES[currentStageIndex];
    const mat = MATERIALS[currentMaterialKey];

    stageTitle.textContent = stage.title;
    stageBadge.textContent = `STAGE ${stage.number}`;
    stageExplanation.textContent = stage.explanation;
    codeFilename.textContent = stage.filename;
    codeSnippetText.textContent = stage.code;

    angleBase.textContent = `${servoAngles.base}°`;
    angleShoulder.textContent = `${servoAngles.shoulder}°`;
    angleElbow.textContent = `${servoAngles.elbow}°`;
    angleWrist1.textContent = `${servoAngles.wrist1}°`;
    angleWrist2.textContent = `${servoAngles.wrist2}°`;
    angleHand.textContent = servoAngles.hand === 180 ? "CLOSED (180°)" : "OPEN (90°)";

    MATERIAL_KEYS.forEach((key) => {
      const rowEl = document.querySelector(`.debounce-row[data-class="${key}"]`);
      if (rowEl) {
        const fill = rowEl.querySelector(".fill");
        const countSpan = rowEl.querySelector(".count-value");
        const cnt = (key === currentMaterialKey && currentStageIndex >= 2) ? consecutiveCounts[key] : 0;
        fill.style.width = `${(cnt / 5) * 100}%`;
        countSpan.textContent = `${cnt}/5`;
        if (key === currentMaterialKey && currentStageIndex >= 2) {
          rowEl.classList.add("active");
        } else {
          rowEl.classList.remove("active");
        }
      }
    });

    renderCameraSvg();
    renderArmSvg();
  }

  // ====== Render Gripper Camera View (SVG) ======
  function renderCameraSvg() {
    const mat = MATERIALS[currentMaterialKey];
    let svgContent = `
      <rect width="300" height="300" fill="var(--canvas-bg)"/>
      <line x1="0" y1="150" x2="300" y2="150" stroke="var(--border-color)" stroke-width="1" stroke-dasharray="2 2"/>
      <line x1="150" y1="0" x2="150" y2="300" stroke="var(--border-color)" stroke-width="1" stroke-dasharray="2 2"/>
    `;

    // Render technical CAD shapes without emojis
    if (mat.name === "Cardboard") {
      svgContent += `
        <rect x="80" y="80" width="140" height="140" fill="${mat.color}" stroke="var(--text-main)" stroke-width="2"/>
        <line x1="80" y1="150" x2="220" y2="150" stroke="var(--border-color)" stroke-width="2"/>
        <text x="150" y="145" fill="#ffffff" font-family="sans-serif" font-weight="bold" font-size="14" text-anchor="middle">CARDBOARD</text>
      `;
    } else if (mat.name === "Glass") {
      svgContent += `
        <path d="M120 70 L180 70 L180 100 L200 130 L200 240 L100 240 L100 130 L120 100 Z" fill="${mat.color}" stroke="var(--text-main)" stroke-width="2"/>
        <rect x="135" y="55" width="30" height="15" fill="var(--border-color)" rx="2"/>
        <text x="150" y="170" fill="#ffffff" font-family="sans-serif" font-weight="bold" font-size="14" text-anchor="middle">GLASS</text>
      `;
    } else if (mat.name === "Metal") {
      svgContent += `
        <rect x="95" y="70" width="110" height="160" rx="15" fill="${mat.color}" stroke="var(--text-main)" stroke-width="2"/>
        <ellipse cx="150" cy="70" rx="55" ry="12" fill="var(--border-color)"/>
        <text x="150" y="155" fill="#ffffff" font-family="sans-serif" font-weight="bold" font-size="14" text-anchor="middle">METAL CAN</text>
      `;
    } else if (mat.name === "Paper") {
      svgContent += `
        <polygon points="90,60 180,60 210,90 210,240 90,240" fill="${mat.color}" stroke="var(--text-main)" stroke-width="2"/>
        <polygon points="180,60 180,90 210,90" fill="var(--border-color)"/>
        <line x1="110" y1="100" x2="170" y2="100" stroke="#ffffff" stroke-width="2"/>
        <line x1="110" y1="130" x2="190" y2="130" stroke="#ffffff" stroke-width="2"/>
        <text x="150" y="180" fill="#ffffff" font-family="sans-serif" font-weight="bold" font-size="14" text-anchor="middle">PAPER SHEET</text>
      `;
    } else if (mat.name === "Plastic") {
      svgContent += `
        <rect x="110" y="60" width="80" height="180" rx="20" fill="${mat.color}" stroke="var(--text-main)" stroke-width="2"/>
        <rect x="130" y="45" width="40" height="15" fill="var(--border-color)" rx="2"/>
        <text x="150" y="160" fill="#ffffff" font-family="sans-serif" font-weight="bold" font-size="14" text-anchor="middle">PLASTIC BOTTLE</text>
      `;
    }

    cameraSvg.innerHTML = svgContent;
  }

  // ====== Render Robotic Arm Workspace Arena (SVG) ======
  function renderArmSvg() {
    const mat = MATERIALS[currentMaterialKey];
    const centerX = 350;
    const centerY = 240;

    let svgContent = `
      <rect width="740" height="480" fill="var(--canvas-bg)"/>
      <circle cx="${centerX}" cy="${centerY}" r="210" fill="none" stroke="var(--border-color)" stroke-dasharray="3 3"/>
      <circle cx="${centerX}" cy="${centerY}" r="140" fill="none" stroke="var(--border-color)" stroke-dasharray="3 3"/>
    `;

    // 1. Render Input Bin Tray holding mixed items (Right side)
    svgContent += `
      <g transform="translate(520, 180)">
        <rect x="0" y="0" width="140" height="120" rx="6" fill="var(--bg-panel)" stroke="var(--border-color)" stroke-width="2"/>
        <text x="70" y="-10" fill="var(--text-muted)" font-size="11" font-weight="bold" font-family="sans-serif" text-anchor="middle">INPUT INTAKE TRAY</text>
        <rect x="15" y="20" width="30" height="30" fill="var(--mat-cardboard)" opacity="0.6" rx="3"/>
        <rect x="55" y="25" width="25" height="40" fill="var(--mat-glass)" opacity="0.6" rx="3"/>
        <rect x="90" y="15" width="35" height="30" fill="var(--mat-metal)" opacity="0.6" rx="3"/>
        <rect x="25" y="65" width="40" height="35" fill="var(--mat-paper)" opacity="0.6" rx="3"/>
        <rect x="75" y="60" width="35" height="45" fill="var(--mat-plastic)" opacity="0.6" rx="3"/>
      </g>
    `;

    // 2. Render 5 Output Bins at Fixed Angles
    MATERIAL_KEYS.forEach((key) => {
      const bMat = MATERIALS[key];
      const rad = (bMat.binAngle - 90) * (Math.PI / 180);
      const binX = centerX + Math.cos(rad) * 175;
      const binY = centerY + Math.sin(rad) * 175;

      svgContent += `
        <g transform="translate(${binX}, ${binY})">
          <rect x="-35" y="-22" width="70" height="44" rx="4" fill="var(--bg-panel)" stroke="${bMat.color}" stroke-width="2"/>
          <text x="0" y="-4" fill="var(--text-main)" font-size="10" font-weight="bold" text-anchor="middle">${bMat.name.toUpperCase()}</text>
          <text x="0" y="10" fill="${bMat.color}" font-size="9" font-family="monospace" text-anchor="middle">${bMat.binAngle}° BIN</text>
        </g>
      `;
    });

    // 3. Render Item Position
    let itemX = 580;
    let itemY = 240;

    if (itemInArena.droppedInBin) {
      const rad = (mat.binAngle - 90) * (Math.PI / 180);
      itemX = centerX + Math.cos(rad) * 175;
      itemY = centerY + Math.sin(rad) * 175;
    } else if (itemInArena.attachedToGripper) {
      const armRad = (servoAngles.base - 90) * (Math.PI / 180);
      itemX = centerX + Math.cos(armRad) * 135;
      itemY = centerY + Math.sin(armRad) * 135;
    }

    svgContent += `
      <g transform="translate(${itemX}, ${itemY})">
        <circle r="14" fill="${mat.color}" stroke="#ffffff" stroke-width="2"/>
      </g>
    `;

    // 4. Render 6-DOF Robotic Arm Joint Kinematics
    const baseRad = (servoAngles.base - 90) * (Math.PI / 180);
    const armLength1 = 80;
    const armLength2 = 60;

    const elbowX = centerX + Math.cos(baseRad) * armLength1;
    const elbowY = centerY + Math.sin(baseRad) * armLength1;

    const gripperX = elbowX + Math.cos(baseRad) * armLength2;
    const gripperY = elbowY + Math.sin(baseRad) * armLength2;

    svgContent += `
      <!-- Base Turret -->
      <circle cx="${centerX}" cy="${centerY}" r="26" fill="var(--bg-panel)" stroke="var(--accent-primary)" stroke-width="3"/>
      <text x="${centerX}" y="${centerY + 4}" fill="var(--text-main)" font-size="9" font-weight="bold" text-anchor="middle">BASE</text>

      <!-- Arm Links -->
      <line x1="${centerX}" y1="${centerY}" x2="${elbowX}" y2="${elbowY}" stroke="var(--accent-primary)" stroke-width="8" stroke-linecap="round"/>
      <circle cx="${elbowX}" cy="${elbowY}" r="10" fill="var(--bg-panel)" stroke="var(--text-main)" stroke-width="2"/>
      <line x1="${elbowX}" y1="${elbowY}" x2="${gripperX}" y2="${gripperY}" stroke="var(--text-muted)" stroke-width="5" stroke-linecap="round"/>

      <!-- Gripper Hand -->
      <g transform="translate(${gripperX}, ${gripperY}) rotate(${servoAngles.base})">
        <path d="M-8,-6 L0,0 L-8,6" fill="none" stroke="${servoAngles.hand === 180 ? 'var(--accent-danger)' : 'var(--accent-success)'}" stroke-width="3" stroke-linecap="round"/>
        <path d="M8,-6 L0,0 L8,6" fill="none" stroke="${servoAngles.hand === 180 ? 'var(--accent-danger)' : 'var(--accent-success)'}" stroke-width="3" stroke-linecap="round"/>
        <circle cx="0" cy="0" r="4" fill="var(--text-main)"/>
      </g>
    `;

    // 5. Render Arduino Microcontroller Board
    svgContent += `
      <g transform="translate(30, 380)">
        <rect width="130" height="70" rx="4" fill="var(--bg-panel)" stroke="var(--border-color)" stroke-width="2"/>
        <text x="65" y="20" fill="var(--text-main)" font-family="monospace" font-weight="bold" font-size="11" text-anchor="middle">ARDUINO UNO</text>
        <circle cx="22" cy="45" r="8" fill="${ledIndicator.classList.contains('led-on') ? 'var(--accent-danger)' : 'var(--border-color)'}" stroke="var(--text-main)" stroke-width="1"/>
        <text x="36" y="49" fill="var(--text-muted)" font-size="10" font-family="monospace">PIN 13 LED</text>
      </g>
    `;

    armSvg.innerHTML = svgContent;
  }

  // Start Engine
  init();
});
