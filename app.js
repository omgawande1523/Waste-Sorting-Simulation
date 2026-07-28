/* ==========================================================================
   AI Waste Segregation Robotic Arm — Single-Frame Topology Engine
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

  // ====== 10 System Pipeline Topology Nodes ======
  const TOPOLOGY_STAGES = [
    {
      id: "FRAME_CAPTURE",
      number: "01",
      title: "FRAME CAPTURE",
      explanation: "Gripper-mounted camera captures raw video matrix of the intake tray looking down.",
      codeRef: "detect_script.py (Line 64)"
    },
    {
      id: "TENS_RESIZE",
      number: "02",
      title: "TENSOR RESIZE (300x300)",
      explanation: "OpenCV resizes raw frame to 300x300 uint8 tensor to fit MobileNet SSD input shape.",
      codeRef: "detect_script.py (Line 68)"
    },
    {
      id: "TFLITE_INFERENCE",
      number: "03",
      title: "TFLITE INFERENCE",
      explanation: "TFLite Interpreter evaluates tensor and outputs bounding boxes, class IDs, and scores.",
      codeRef: "detect_script.py (Lines 71-76)"
    },
    {
      id: "CONFIDENCE_CHECK",
      number: "04",
      title: "CONFIDENCE CHECK (>0.5)",
      explanation: "Filters detections against 0.5 threshold probability to eliminate low-certainty noise.",
      codeRef: "detect_script.py (Line 80)"
    },
    {
      id: "DEBOUNCE_FILTER",
      number: "05",
      title: "DEBOUNCE COUNTER (5x)",
      explanation: "Requires 5 CONSECUTIVE confident matches for a material class before triggering physical arm.",
      codeRef: "detect_script.py (Line 97)"
    },
    {
      id: "SERIAL_SEND",
      number: "06",
      title: "SERIAL TX (PI → ARDUINO)",
      explanation: "Raspberry Pi sends formatted serial packet (code, dist, angle) over COM7 @ 9600 baud.",
      codeRef: "detect_script.py (Lines 100-108)"
    },
    {
      id: "ARDUINO_LED_FLASH",
      number: "07",
      title: "SERIAL RX & LED FLASH",
      explanation: "Arduino receives packet via Serial.parseInt() and flashes Pin 13 LED N times in confirmation.",
      codeRef: "newprogramwaste.ino (Lines 73-79)"
    },
    {
      id: "ARM_PICKUP",
      number: "08",
      title: "ARM PICKUP INTERPOLATION",
      explanation: "Arduino calls pickUp(), sweeping joint servos degree-by-degree (30ms) to grip intake item.",
      codeRef: "newprogramwaste.ino (Lines 100-133)"
    },
    {
      id: "ARM_TRANSIT_DROPOFF",
      number: "09",
      title: "BIN TRANSIT & DROPOFF",
      explanation: "Base servo rotates to fixed output bin angle (0°-180°) and opens gripper claw to release item.",
      codeRef: "newprogramwaste.ino (Lines 205-280)"
    },
    {
      id: "HOME_RESET",
      number: "10",
      title: "HOME RESET & DONE REPLY",
      explanation: "Servos return to resting pose facing intake tray; Arduino sends 'Done Moving' back to Pi.",
      codeRef: "newprogramwaste.ino (Lines 87-88)"
    }
  ];

  // ====== State Engine ======
  let currentStageIndex = 0;
  let isPlaying = false;
  let speedMultiplier = 1.0;
  let manualOverride = "auto";
  let currentMaterialKey = "cardboard";
  let currentItemIndex = 0;
  let consecutiveCounts = { cardboard: 0, glass: 0, metal: 0, paper: 0, plastic: 0 };
  let simulationTimer = null;
  let currentTheme = localStorage.getItem("theme") || "dark";

  // Servo positions
  let servoAngles = { base: 90, shoulder: 65, elbow: 110, wrist1: 90, wrist2: 60, hand: 90 };
  let itemInArena = { x: 580, y: 220, attachedToGripper: false, droppedInBin: false };

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

  const topologyNodes = document.getElementById("topologyNodes");
  const topologyExplainerText = document.getElementById("topologyExplainerText");

  const angleBase = document.getElementById("angleBase");
  const angleShoulder = document.getElementById("angleShoulder");
  const angleElbow = document.getElementById("angleElbow");
  const angleWrist1 = document.getElementById("angleWrist1");
  const angleWrist2 = document.getElementById("angleWrist2");
  const angleHand = document.getElementById("angleHand");

  // ====== Initializer ======
  function init() {
    applyTheme(currentTheme);
    renderTopologyNodes();
    setupEventListeners();
    renderCameraSvg();
    renderArmSvg();
    executeCurrentStage();
  }

  function renderTopologyNodes() {
    topologyNodes.innerHTML = "";
    TOPOLOGY_STAGES.forEach((stage, idx) => {
      const node = document.createElement("div");
      node.className = "topo-node";
      node.setAttribute("data-index", idx);
      node.innerHTML = `
        <span class="node-number">${stage.number}</span>
        <span class="node-title">${stage.title}</span>
        <span class="node-badge">WAIT</span>
      `;
      node.addEventListener("mouseenter", () => {
        topologyExplainerText.textContent = `${stage.explanation} [${stage.codeRef}]`;
      });
      node.addEventListener("mouseleave", () => {
        const currentStage = TOPOLOGY_STAGES[currentStageIndex];
        topologyExplainerText.textContent = `${currentStage.explanation} [${currentStage.codeRef}]`;
      });
      node.addEventListener("click", () => {
        if (isPlaying) togglePlayPause();
        currentStageIndex = idx;
        executeCurrentStage();
      });
      topologyNodes.appendChild(node);
    });
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
      playSvg.innerHTML = `<rect x="5" y="4" width="4" height="16"></rect><rect x="15" y="4" width="4" height="16"></rect>`;
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

    const stageDuration = Math.max(350, 1200 / speedMultiplier);
    simulationTimer = setTimeout(() => {
      if (currentStageIndex < TOPOLOGY_STAGES.length - 1) {
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
    if (currentStageIndex < TOPOLOGY_STAGES.length - 1) {
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
    itemInArena.y = 220;
    itemInArena.attachedToGripper = false;
    itemInArena.droppedInBin = false;
  }

  // ====== Execute Pipeline Stage ======
  function executeCurrentStage() {
    const stage = TOPOLOGY_STAGES[currentStageIndex];
    const mat = MATERIALS[currentMaterialKey];

    if (stage.id !== "ARDUINO_LED_FLASH") {
      ledIndicator.classList.remove("led-on");
      ledIndicator.textContent = "PIN 13 LED: OFF";
    }

    switch (stage.id) {
      case "FRAME_CAPTURE":
        detectionBox.classList.add("hidden");
        break;

      case "TENS_RESIZE":
      case "TFLITE_INFERENCE":
      case "CONFIDENCE_CHECK":
        detectionBox.classList.remove("hidden");
        const b = mat.box;
        detectionBox.style.left = `${b.xmin * 100}%`;
        detectionBox.style.top = `${b.ymin * 100}%`;
        detectionBox.style.width = `${(b.xmax - b.xmin) * 100}%`;
        detectionBox.style.height = `${(b.ymax - b.ymin) * 100}%`;
        boxLabel.textContent = `${mat.name.toUpperCase()} [CONF: ${mat.confidence.toFixed(2)}]`;
        break;

      case "DEBOUNCE_FILTER":
        consecutiveCounts[currentMaterialKey] = 5;
        debounceStatusText.textContent = `5 / 5 CONFIRMED`;
        break;

      case "SERIAL_SEND":
        logSerial(`[PI → ARDUINO] PACKET SENT: Code=${mat.code} (${mat.name.toUpperCase()}), Dist=1, Angle=${mat.binAngle}°`, "tx");
        break;

      case "ARDUINO_LED_FLASH":
        ledIndicator.classList.add("led-on");
        ledIndicator.textContent = `PIN 13 LED: FLASH (${mat.code}X)`;
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

      case "HOME_RESET":
        servoAngles = { base: 90, shoulder: 65, elbow: 110, wrist1: 90, wrist2: 60, hand: 90 };
        logSerial(`[ARDUINO → PI] TX: "Done Moving"`, "highlight");
        break;
    }

    updateTopologyUI();
    updateUI();
  }

  // ====== Update Topology Node Graph Highlights ======
  function updateTopologyUI() {
    const currentStage = TOPOLOGY_STAGES[currentStageIndex];
    topologyExplainerText.textContent = `${currentStage.explanation} [${currentStage.codeRef}]`;

    const nodes = topologyNodes.querySelectorAll(".topo-node");
    nodes.forEach((node, idx) => {
      const badge = node.querySelector(".node-badge");
      node.classList.remove("active", "done");

      if (idx === currentStageIndex) {
        node.classList.add("active");
        badge.textContent = "RUNNING";
      } else if (idx < currentStageIndex) {
        node.classList.add("done");
        badge.textContent = "DONE";
      } else {
        badge.textContent = "WAIT";
      }
    });
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

  // ====== Update UI Labels & Telemetry ======
  function updateUI() {
    const mat = MATERIALS[currentMaterialKey];

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
        const cnt = (key === currentMaterialKey && currentStageIndex >= 4) ? consecutiveCounts[key] : 0;
        fill.style.width = `${(cnt / 5) * 100}%`;
        countSpan.textContent = `${cnt}/5`;
        if (key === currentMaterialKey && currentStageIndex >= 4) {
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
    const centerY = 220;

    let svgContent = `
      <rect width="740" height="440" fill="var(--canvas-bg)"/>
      <circle cx="${centerX}" cy="${centerY}" r="190" fill="none" stroke="var(--border-color)" stroke-dasharray="3 3"/>
      <circle cx="${centerX}" cy="${centerY}" r="130" fill="none" stroke="var(--border-color)" stroke-dasharray="3 3"/>
    `;

    // 1. Render Input Bin Tray holding mixed items
    svgContent += `
      <g transform="translate(520, 160)">
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
      const binX = centerX + Math.cos(rad) * 165;
      const binY = centerY + Math.sin(rad) * 165;

      svgContent += `
        <g transform="translate(${binX}, ${binY})">
          <rect x="-35" y="-20" width="70" height="40" rx="4" fill="var(--bg-panel)" stroke="${bMat.color}" stroke-width="2"/>
          <text x="0" y="-3" fill="var(--text-main)" font-size="10" font-weight="bold" text-anchor="middle">${bMat.name.toUpperCase()}</text>
          <text x="0" y="9" fill="${bMat.color}" font-size="9" font-family="monospace" text-anchor="middle">${bMat.binAngle}° BIN</text>
        </g>
      `;
    });

    // 3. Render Item Position
    let itemX = 580;
    let itemY = 220;

    if (itemInArena.droppedInBin) {
      const rad = (mat.binAngle - 90) * (Math.PI / 180);
      itemX = centerX + Math.cos(rad) * 165;
      itemY = centerY + Math.sin(rad) * 165;
    } else if (itemInArena.attachedToGripper) {
      const armRad = (servoAngles.base - 90) * (Math.PI / 180);
      itemX = centerX + Math.cos(armRad) * 130;
      itemY = centerY + Math.sin(armRad) * 130;
    }

    svgContent += `
      <g transform="translate(${itemX}, ${itemY})">
        <circle r="13" fill="${mat.color}" stroke="#ffffff" stroke-width="2"/>
      </g>
    `;

    // 4. Render 6-DOF Robotic Arm Joint Kinematics
    const baseRad = (servoAngles.base - 90) * (Math.PI / 180);
    const armLength1 = 75;
    const armLength2 = 55;

    const elbowX = centerX + Math.cos(baseRad) * armLength1;
    const elbowY = centerY + Math.sin(baseRad) * armLength1;

    const gripperX = elbowX + Math.cos(baseRad) * armLength2;
    const gripperY = elbowY + Math.sin(baseRad) * armLength2;

    svgContent += `
      <!-- Base Turret -->
      <circle cx="${centerX}" cy="${centerY}" r="24" fill="var(--bg-panel)" stroke="var(--accent-primary)" stroke-width="3"/>
      <text x="${centerX}" y="${centerY + 4}" fill="var(--text-main)" font-size="9" font-weight="bold" text-anchor="middle">BASE</text>

      <!-- Arm Links -->
      <line x1="${centerX}" y1="${centerY}" x2="${elbowX}" y2="${elbowY}" stroke="var(--accent-primary)" stroke-width="7" stroke-linecap="round"/>
      <circle cx="${elbowX}" cy="${elbowY}" r="9" fill="var(--bg-panel)" stroke="var(--text-main)" stroke-width="2"/>
      <line x1="${elbowX}" y1="${elbowY}" x2="${gripperX}" y2="${gripperY}" stroke="var(--text-muted)" stroke-width="5" stroke-linecap="round"/>

      <!-- Gripper Hand -->
      <g transform="translate(${gripperX}, ${gripperY}) rotate(${servoAngles.base})">
        <path d="M-8,-5 L0,0 L-8,5" fill="none" stroke="${servoAngles.hand === 180 ? 'var(--accent-danger)' : 'var(--accent-success)'}" stroke-width="3" stroke-linecap="round"/>
        <path d="M8,-5 L0,0 L8,5" fill="none" stroke="${servoAngles.hand === 180 ? 'var(--accent-danger)' : 'var(--accent-success)'}" stroke-width="3" stroke-linecap="round"/>
        <circle cx="0" cy="0" r="4" fill="var(--text-main)"/>
      </g>
    `;

    // 5. Render Arduino Microcontroller Board
    svgContent += `
      <g transform="translate(25, 350)">
        <rect width="125" height="65" rx="4" fill="var(--bg-panel)" stroke="var(--border-color)" stroke-width="2"/>
        <text x="62" y="18" fill="var(--text-main)" font-family="monospace" font-weight="bold" font-size="10" text-anchor="middle">ARDUINO UNO</text>
        <circle cx="20" cy="42" r="7" fill="${ledIndicator.classList.contains('led-on') ? 'var(--accent-danger)' : 'var(--border-color)'}" stroke="var(--text-main)" stroke-width="1"/>
        <text x="34" y="46" fill="var(--text-muted)" font-size="9" font-family="monospace">PIN 13 LED</text>
      </g>
    `;

    armSvg.innerHTML = svgContent;
  }

  // Start Engine
  init();
});
