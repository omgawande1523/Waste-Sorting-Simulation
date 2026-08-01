/* ==========================================================================
   AI Waste Segregation Robotic Arm — Full System Architecture Mesh Simulation Engine
   Grounded in detect_script.py (8-Frame Decision Policy, Pose Estimator)
   & newprogramwaste.ino (Non-Blocking Servos)
   ========================================================================== */

document.addEventListener("DOMContentLoaded", () => {
  // ====== Material Definitions with Distinct Bounding Box Coordinates ======
  // Fast Path: Cardboard (0.92), Metal (0.94), Plastic (0.95)
  // Ambiguous Path: Glass (0.72 - Glass/Plastic Ambiguity)
  const MATERIALS = {
    cardboard: {
      name: "Cardboard",
      code: 1,
      color: "var(--mat-cardboard)",
      binAngle: 0,
      confidence: 0.92,
      box: { xmin: 0.10, ymin: 0.15, xmax: 0.40, ymax: 0.45 }
    },
    glass: {
      name: "Glass",
      code: 2,
      color: "var(--mat-glass)",
      binAngle: 45,
      confidence: 0.72, // Ambiguous path trigger
      box: { xmin: 0.60, ymin: 0.15, xmax: 0.85, ymax: 0.45 }
    },
    metal: {
      name: "Metal",
      code: 3,
      color: "var(--mat-metal)",
      binAngle: 90,
      confidence: 0.94,
      box: { xmin: 0.42, ymin: 0.42, xmax: 0.58, ymax: 0.58 }
    },
    paper: {
      name: "Paper",
      code: 4,
      color: "var(--mat-paper)",
      binAngle: 135,
      confidence: 0.88, // Fast path
      box: { xmin: 0.15, ymin: 0.55, xmax: 0.45, ymax: 0.85 }
    },
    plastic: {
      name: "Plastic",
      code: 5,
      color: "var(--mat-plastic)",
      binAngle: 180,
      confidence: 0.95,
      box: { xmin: 0.65, ymin: 0.40, xmax: 0.85, ymax: 0.70 }
    }
  };

  const MATERIAL_KEYS = Object.keys(MATERIALS);

  // ====== Firmware Joint Servo Angles per Distance Zone (from newprogramwaste.ino) ======
  const ZONE_TARGETS = {
    1: { wrist2: 35, elbow: 180, shoulder: 65 },
    2: { wrist2: 37, elbow: 175, shoulder: 60 },
    3: { wrist2: 32, elbow: 165, shoulder: 55 },
    4: { wrist2: 32, elbow: 160, shoulder: 50 },
    5: { wrist2: 32, elbow: 150, shoulder: 45 }
  };

  const HOME_TARGETS = { base: 90, shoulder: 65, elbow: 110, wrist1: 90, wrist2: 60, hand: 90 };

  // ====== Full System Architecture Mesh Topology Nodes ======
  const MESH_NODES = [
    {
      id: "COARSE_DETECTOR",
      stageIndex: 0,
      number: "01",
      title: "Coarse Detector Agent",
      sub: "Always-On Fast Detection",
      x: 40, y: 165, w: 165, h: 80,
      hasRl: false
    },
    {
      id: "CONFIDENCE_GATE",
      stageIndex: 1,
      number: "02",
      title: "Confidence Gate",
      sub: "Certainty Router (≥0.80)",
      x: 245, y: 165, w: 160, h: 80,
      hasRl: false
    },
    {
      id: "SPECIALIST_AGENTS",
      stageIndex: 2,
      number: "03",
      title: "Specialist Agents",
      sub: "Domain Expert Classifier",
      isGroup: true,
      x: 435, y: 215, w: 235, h: 120,
      hasRl: false,
      subNodes: [
        {
          id: "SPEC_GLASS_PLASTIC",
          title: "Glass vs Plastic Specialist",
          sub: "Reflectance & Specular Classifier",
          x: 445, y: 245, w: 215, h: 75
        }
      ]
    },
    {
      id: "ARBITRATION_AGENT",
      stageIndex: 3,
      number: "04",
      title: "Arbitration Agent",
      sub: "Material Commitment",
      x: 680, y: 165, w: 165, h: 80,
      hasRl: true
    },
    {
      id: "MOTION_AGENT",
      stageIndex: 4,
      number: "05",
      title: "Motion Agent",
      sub: "6-DOF Kinematic Controller",
      x: 885, y: 165, w: 160, h: 80,
      hasRl: true
    },
    {
      id: "VERIFY_AGENT",
      stageIndex: 5,
      number: "06",
      title: "Verify Agent",
      sub: "Bin Placement Check",
      x: 1085, y: 165, w: 140, h: 80,
      hasRl: false
    },
    {
      id: "RETRAINING_JOB",
      stageIndex: 6,
      number: "07",
      title: "Retraining Job",
      sub: "Continuous Learning Loop",
      x: 885, y: 355, w: 160, h: 75,
      hasRl: false
    }
  ];

  // ====== Layer Modal Details Mapping ======
  const NODE_DETAILS = {
    "COARSE_DETECTOR": {
      number: "01",
      title: "Coarse Detector Agent",
      explanation: "Lightweight MobileNet SSD model continuously processes camera frames over intake workspace.",
      rationale: "Performs high-throughput single-pass visual detection on 300x300 video frames at 30+ FPS to locate objects in the workspace.",
      pathNote: "PATH TYPE: Active on both Fast Path (≥0.80) and Ambiguous Path (<0.80).",
      dataSpec: "Input: 300x300x3 RGB video tensor → Output: BBoxes [ymin, xmin, ymax, xmax], Class ID, Conf score",
      reference: "Corresponds to cap.read() & interpreter.invoke() in detect_script.py (Line 685)"
    },
    "CONFIDENCE_GATE": {
      number: "02",
      title: "Confidence Gate",
      explanation: "Routes high-certainty detections (≥0.80) directly to Arbitration, and low-certainty detections (<0.80) to Specialist Agents.",
      rationale: "Optimizes compute latency by bypassing secondary classification for clear items while isolating ambiguous items for specialist routing.",
      pathNote: "PATH TYPE: Decision router for Fast Path (≥0.80) vs Ambiguous Path (<0.80).",
      dataSpec: "Threshold check: Conf ≥ 0.80 → FAST_PATH | Conf < 0.80 → AMBIGUOUS_PATH",
      reference: "Corresponds to CONFIDENCE_THRESHOLD & DecisionPolicy in detect_script.py"
    },
    "SPECIALIST_AGENTS": {
      number: "03",
      title: "Specialist Agents (Domain Experts)",
      explanation: "Parallel domain-expert neural networks invoked exclusively for ambiguous path items to resolve fine-grained material ambiguity.",
      rationale: "Specialized models analyze specular reflection patterns and micro-textures to disambiguate transparent plastic from glass.",
      pathNote: "PATH TYPE: Ambiguous Path ONLY (Stays dim/inactive during Fast Path cycles).",
      dataSpec: "Input: Sub-region ROI crop → Output: Refined Class Probability Logits",
      reference: "Corresponds to specialist.py & test_specialist_trigger.py"
    },
    "SPEC_GLASS_PLASTIC": {
      number: "03",
      title: "Glass vs Plastic Specialist",
      explanation: "Targeted secondary classifier evaluating specular highlights and surface reflection variance.",
      rationale: "Specialized CNN analyzing specular reflection patterns and transparency indices to disambiguate clear PET plastic from glass bottles.",
      pathNote: "PATH TYPE: Ambiguous Path ONLY (Invoked when material is Glass or Plastic with confidence < 0.80).",
      dataSpec: "Input: BBox Specular Crop → Output: Glass Logits vs Plastic Logits",
      reference: "Corresponds to specialist_model.pt & test_specialist_trigger.py"
    },
    "ARBITRATION_AGENT": {
      number: "04",
      title: "Arbitration Agent [RL]",
      explanation: "Reinforcement learning agent that synthesizes detection certainty and specialist logits to make the final commitment decision.",
      rationale: "Evaluates multi-source evidence and historical reward policy to commit to a material class and compute base pick-up pose.",
      pathNote: "PATH TYPE: Active on both Fast Path (direct from Gate) and Ambiguous Path (via Specialists).",
      dataSpec: "State: [Conf, BBox, Specialist Logits] → Action: [Final Material Class, Base Angle 0-180°, Distance Zone 1-5]",
      reference: "Corresponds to decision_policy.json & learned_policy.py"
    },
    "MOTION_AGENT": {
      number: "05",
      title: "Motion Agent [RL]",
      explanation: "RL motion controller converting target pose into non-blocking 6-DOF servo trajectory angles.",
      rationale: "Computes base turret rotation and joint reach angles (Zone 1-5 targets), dispatching serial packet commands to Arduino Uno.",
      pathNote: "PATH TYPE: Active on both Fast Path and Ambiguous Path.",
      dataSpec: "Packet TX: \"num_blink material distance angle\\n\" → Servo joint targets",
      reference: "Corresponds to pickUp() & dropOff() in newprogramwaste.ino"
    },
    "VERIFY_AGENT": {
      number: "06",
      title: "Verify Agent",
      explanation: "Post-dropoff verification agent confirming physical release and placement into destination bin.",
      rationale: "Validates gripper release and item arrival in destination bin, logging sorting confirmation.",
      pathNote: "PATH TYPE: Active on both Fast Path and Ambiguous Path.",
      dataSpec: "Verification Signal: ITEM_DROPPED_SUCCESS = true → Sorting log record",
      reference: "Corresponds to wait_for_done_moving() & sorting_log.csv"
    },
    "RETRAINING_JOB": {
      number: "07",
      title: "Retraining Job (Continuous Feedback Loop)",
      explanation: "Asynchronous learning pipeline consuming sorting telemetry to retrain model weights.",
      rationale: "Feeds verification records back into offline training scripts to update coarse detector and domain specialist neural weights.",
      pathNote: "PATH TYPE: Feedback Return Loop (Loops back to Node 01 Coarse Detector & Node 03 Specialists).",
      dataSpec: "Input: Verified Telemetry Log → Output: Updated Model Weights (detect.tflite, specialist.pt)",
      reference: "Corresponds to retrain_from_log.py & train_policy.py"
    }
  };

  // ====== State Engine ======
  let currentStageIndex = 0;
  let isPlaying = false;
  let speedMultiplier = 1.0;
  let manualOverride = "auto";
  let currentMaterialKey = "cardboard";
  let currentItemIndex = 0;
  let simulationTimer = null;
  let currentTheme = localStorage.getItem("theme") || "dark";
  let isMeshExpanded = false;

  // 8-Frame Sliding Window Deque
  let slidingWindow = [];

  // Servo positions
  let servoAngles = { ...HOME_TARGETS };
  let itemInArena = { x: 610, y: 220, attachedToGripper: false, droppedInBin: false };

  // ====== DOM Selectors ======
  const btnPlayPause = document.getElementById("btnPlayPause");
  const playSvg = document.getElementById("playSvg");
  const playText = document.getElementById("playText");
  const btnPrev = document.getElementById("btnPrev");
  const btnNext = document.getElementById("btnNext");
  const btnRestart = document.getElementById("btnRestart");

  // Panel 3 Header Playback & Speed Controls
  const btnPanel3PlayPause = document.getElementById("btnPanel3PlayPause");
  const panel3PlaySvg = document.getElementById("panel3PlaySvg");
  const panel3PlayText = document.getElementById("panel3PlayText");
  const btnPanel3Prev = document.getElementById("btnPanel3Prev");
  const btnPanel3Next = document.getElementById("btnPanel3Next");
  const btnPanel3Restart = document.getElementById("btnPanel3Restart");
  const panel3SpeedSlider = document.getElementById("panel3SpeedSlider");
  const panel3SpeedVal = document.getElementById("panel3SpeedVal");

  const speedSlider = document.getElementById("speedSlider");
  const speedVal = document.getElementById("speedVal");
  const itemSelect = document.getElementById("itemSelect");
  const themeToggle = document.getElementById("themeToggle");
  const themeLabelText = document.getElementById("themeLabelText");
  const btnClearSerial = document.getElementById("btnClearSerial");

  const btnExpandMesh = document.getElementById("btnExpandMesh");
  const expandSvg = document.getElementById("expandSvg");
  const topologyPanel = document.getElementById("topologyPanel");
  const meshSvg = document.getElementById("meshSvg");
  const topologyExplainerText = document.getElementById("topologyExplainerText");

  const cameraSvg = document.getElementById("cameraSvg");
  const detectionBox = document.getElementById("detectionBox");
  const boxLabel = document.getElementById("boxLabel");
  const poseAngleText = document.getElementById("poseAngleText");
  const poseZoneText = document.getElementById("poseZoneText");
  const poseRadiusText = document.getElementById("poseRadiusText");

  const windowSlotsContainer = document.getElementById("windowSlotsContainer");
  const policyOutcomeText = document.getElementById("policyOutcomeText");

  const armSvg = document.getElementById("armSvg");
  const ledIndicator = document.getElementById("ledIndicator");
  const packetTag = document.getElementById("packetTag");
  const serialLog = document.getElementById("serialLog");

  // Modal Selectors
  const infoModalOverlay = document.getElementById("infoModalOverlay");
  const modalBadge = document.getElementById("modalBadge");
  const modalTitle = document.getElementById("modalTitle");
  const modalRationale = document.getElementById("modalRationale");
  const modalPathNote = document.getElementById("modalPathNote");
  const modalDataSpec = document.getElementById("modalDataSpec");
  const modalReference = document.getElementById("modalReference");
  const btnModalClose = document.getElementById("btnModalClose");

  const angleBase = document.getElementById("angleBase");
  const angleShoulder = document.getElementById("angleShoulder");
  const angleElbow = document.getElementById("angleElbow");
  const angleWrist1 = document.getElementById("angleWrist1");
  const angleWrist2 = document.getElementById("angleWrist2");
  const angleHand = document.getElementById("angleHand");

  // ====== Bounding Box Pose Estimation Helper (exact math from detect_script.py) ======
  function calculatePose(box) {
    const centre_x = (box.xmin + box.xmax) / 2.0;
    const centre_y = (box.ymin + box.ymax) / 2.0;
    
    // Angle: linear map from centre_x to 0..180
    const rawAngle = 0 + centre_x * 180.0;
    const baseAngle = Math.round(Math.max(0, Math.min(180, rawAngle)));

    // Radial distance from pivot (0.5, 0.5)
    const dx = (centre_x - 0.5) / 0.5;
    const dy = (centre_y - 0.5) / 0.5;
    const radius = Math.sqrt(dx * dx + dy * dy);

    // Distance zones based on ZONE_RADIUS_EDGES = (0.25, 0.50, 0.75, 1.00)
    let zone = 5;
    const edges = [0.25, 0.50, 0.75, 1.00];
    for (let i = 0; i < edges.length; i++) {
      if (radius < edges[i]) {
        zone = i + 1;
        break;
      }
    }

    return { centre_x, centre_y, radius, baseAngle, zone };
  }

  // ====== Initializer ======
  function init() {
    applyTheme(currentTheme);
    setupEventListeners();
    resetSlidingWindow();
    renderCameraSvg();
    renderArmSvg();
    renderMeshSvg();
    executeCurrentStage();
  }

  function resetSlidingWindow() {
    slidingWindow = [];
    renderWindowSlots();
  }

  function isCurrentItemFastPath() {
    const mat = MATERIALS[currentMaterialKey];
    return mat.confidence >= 0.80;
  }

  // ====== Render Mesh SVG Topology Diagram ======
  function renderMeshSvg() {
    const isFast = isCurrentItemFastPath();
    const activeStage = currentStageIndex;

    let svgHtml = `
      <defs>
        <!-- Marker Arrows -->
        <marker id="arrow-default" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M 0 1 L 10 5 L 0 9 z" fill="var(--border-color)"/>
        </marker>
        <marker id="arrow-active" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M 0 1 L 10 5 L 0 9 z" fill="var(--accent-primary)"/>
        </marker>
        <marker id="arrow-done" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M 0 1 L 10 5 L 0 9 z" fill="var(--accent-success)"/>
        </marker>
        <marker id="arrow-feedback" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="6" markerHeight="6" orient="auto-start-reverse">
          <path d="M 0 1 L 10 5 L 0 9 z" fill="#c084fc"/>
        </marker>
      </defs>

      <!-- Background Canvas Grid -->
      <rect width="1260" height="510" fill="var(--canvas-bg)" rx="4"/>
    `;

    // Edge State Helper
    function getEdgeState(fromStage, toStage, isPathCondition = true) {
      if (!isPathCondition) return "mesh-edge";
      if (activeStage === toStage) return "mesh-edge active";
      if (activeStage > toStage) return "mesh-edge done";
      if (activeStage === fromStage) return "mesh-edge active";
      return "mesh-edge";
    }

    // 1. Edge: Node 01 -> Node 02 (Coarse Detector -> Confidence Gate)
    const edge1_2 = getEdgeState(0, 1);
    const m1_2 = edge1_2.includes("active") ? "url(#arrow-active)" : (edge1_2.includes("done") ? "url(#arrow-done)" : "url(#arrow-default)");
    svgHtml += `<path d="M 205 205 L 265 205" class="${edge1_2}" marker-end="${m1_2}"/>`;

    // 2. Edge: Direct Fast Path Node 02 -> Node 04 (Confidence Gate -> Arbitration Agent)
    const edgeFast = getEdgeState(1, 3, isFast);
    const mFast = edgeFast.includes("active") ? "url(#arrow-active)" : (edgeFast.includes("done") ? "url(#arrow-done)" : "url(#arrow-default)");
    svgHtml += `
      <path d="M 405 185 L 680 185" class="${edgeFast}" marker-end="${mFast}"/>
      <g class="edge-label-group">
        <rect x="490" y="165" width="105" height="18" fill="var(--canvas-bg)" stroke="var(--border-color)" rx="3" opacity="0.92"/>
        <text x="542.5" y="177" text-anchor="middle" class="edge-label fast-label">FAST PATH (≥0.80)</text>
      </g>
    `;

    // 3. Edges: Ambiguous Path Branching Node 02 -> Specialist 03
    const edgeAmbBranch = getEdgeState(1, 2, !isFast);
    const mAmbBranch = edgeAmbBranch.includes("active") ? "url(#arrow-active)" : (edgeAmbBranch.includes("done") ? "url(#arrow-done)" : "url(#arrow-default)");
    svgHtml += `
      <path d="M 405 225 C 405 270, 340 282.5, 445 282.5" class="${edgeAmbBranch}" marker-end="${mAmbBranch}"/>
      <g class="edge-label-group">
        <rect x="296" y="273.5" width="98" height="18" fill="var(--canvas-bg)" stroke="var(--border-color)" rx="3" opacity="0.92"/>
        <text x="345" y="285.5" text-anchor="middle" class="edge-label amb-label">AMBIGUOUS (&lt;0.80)</text>
      </g>
    `;

    // 4. Edges: Specialist 03 -> Node 04 (Arbitration Agent)
    const edgeSpecConv = getEdgeState(2, 3, !isFast);
    const mSpecConv = edgeSpecConv.includes("active") ? "url(#arrow-active)" : (edgeSpecConv.includes("done") ? "url(#arrow-done)" : "url(#arrow-default)");
    svgHtml += `
      <path d="M 660 282.5 C 672 282.5, 672 225, 680 225" class="${edgeSpecConv}" marker-end="${mSpecConv}"/>
    `;

    // 5. Edge: Node 04 -> Node 05 (Arbitration Agent -> Motion Agent)
    const edge4_5 = getEdgeState(3, 4);
    const m4_5 = edge4_5.includes("active") ? "url(#arrow-active)" : (edge4_5.includes("done") ? "url(#arrow-done)" : "url(#arrow-default)");
    svgHtml += `<path d="M 845 205 L 885 205" class="${edge4_5}" marker-end="${m4_5}"/>`;

    // 6. Edge: Node 05 -> Node 06 (Motion Agent -> Verify Agent)
    const edge5_6 = getEdgeState(4, 5);
    const m5_6 = edge5_6.includes("active") ? "url(#arrow-active)" : (edge5_6.includes("done") ? "url(#arrow-done)" : "url(#arrow-default)");
    svgHtml += `<path d="M 1045 205 L 1085 205" class="${edge5_6}" marker-end="${m5_6}"/>`;

    // 7. Edge: Node 06 -> Node 07 (Verify Agent -> Retraining Job)
    const edge6_7 = getEdgeState(5, 6);
    const m6_7 = edge6_7.includes("active") ? "url(#arrow-active)" : (edge6_7.includes("done") ? "url(#arrow-done)" : "url(#arrow-default)");
    svgHtml += `<path d="M 1155 245 L 1155 392 L 1045 392" class="${edge6_7}" marker-end="${m6_7}"/>`;

    // 8. Retraining Feedback Return Path (Node 07 -> Node 01 & Node 03)
    const isFeedbackActive = (activeStage === 6);
    const feedbackClass = isFeedbackActive ? "mesh-edge feedback active" : "mesh-edge feedback";
    const mFeedback = "url(#arrow-feedback)";

    svgHtml += `
      <!-- Main return trunk line along bottom (y=450) -->
      <path d="M 885 392 L 885 450 L 122 450 L 122 245" class="${feedbackClass}" marker-end="${mFeedback}"/>
      <!-- Branching loop return line to Specialist Group (x=552) -->
      <path d="M 552 450 L 552 335" class="${feedbackClass}" marker-end="${mFeedback}"/>
      <g class="edge-label-group">
        <rect x="362" y="466" width="360" height="18" fill="var(--canvas-bg)" stroke="var(--border-color)" rx="3" opacity="0.92"/>
        <text x="542" y="478" text-anchor="middle" class="edge-label feedback-label">↺ RETRAINING FEEDBACK LOOP (WEIGHT &amp; POLICY UPDATE)</text>
      </g>
    `;

    // Render Specialist Agents Outer Grouping Box (Node 03 Group)
    const specActive = (activeStage === 2 && !isFast);
    const specDone = (activeStage > 2 && !isFast);
    const groupClass = specActive ? "mesh-group-rect active" : "mesh-group-rect";

    svgHtml += `
      <g class="mesh-group" id="node-03" onclick="window.openSpecificModal('SPECIALIST_AGENTS')">
        <rect x="435" y="215" width="235" height="120" class="${groupClass}"/>
        <text x="445" y="235" class="mesh-node-num">03 | SPECIALIST AGENTS</text>
      </g>
    `;

    // Render Sub-nodes inside Specialist Box
    MESH_NODES[2].subNodes.forEach((sub) => {
      const subState = specActive ? "active" : (specDone ? "done" : "");
      const specKey = sub.id;

      let subBadgeText = "WAIT";
      if (subState === "active") subBadgeText = "RUNNING";
      if (subState === "done") subBadgeText = "DONE";

      svgHtml += `
        <g class="mesh-node ${subState}" onclick="event.stopPropagation(); window.openSpecificModal('${specKey}')">
          <rect x="${sub.x}" y="${sub.y}" width="${sub.w}" height="${sub.h}" class="mesh-node-rect"/>
          <text x="${sub.x + 8}" y="${sub.y + 18}" class="mesh-node-title" style="font-size: 10px; font-weight: 600; letter-spacing: -0.1px;">03. ${sub.title}</text>
          <text x="${sub.x + 8}" y="${sub.y + 35}" class="mesh-node-sub" style="font-size: 8.5px;">${sub.sub}</text>
          <text x="${sub.x + 8}" y="${sub.y + 53}" class="mesh-node-status" style="font-size: 8.5px; fill: ${subState === 'active' ? 'var(--accent-primary)' : (subState === 'done' ? 'var(--accent-success)' : 'var(--text-subtle)')};">${subBadgeText}</text>
          <text x="${sub.x + sub.w - 18}" y="${sub.y + 53}" class="mesh-node-info">[i]</text>
        </g>
      `;
    });

    // Render Primary Nodes (01, 02, 04, 05, 06, 07)
    MESH_NODES.forEach((node, nIdx) => {
      if (node.isGroup) return; // Handled above

      let nodeState = "";
      if (nIdx === activeStage) {
        nodeState = "active";
      } else if (nIdx < activeStage) {
        if (nIdx === 2 && isFast) {
          nodeState = ""; // Stays dim/inactive on fast path!
        } else {
          nodeState = "done";
        }
      }

      let badgeText = "WAIT";
      if (nodeState === "active") badgeText = "RUNNING";
      if (nodeState === "done") badgeText = "DONE";

      svgHtml += `
        <g class="mesh-node ${nodeState}" id="node-${node.number}" onclick="window.openSpecificModal('${node.id}')">
          <rect x="${node.x}" y="${node.y}" width="${node.w}" height="${node.h}" class="mesh-node-rect"/>
          <text x="${node.x + 10}" y="${node.y + 18}" class="mesh-node-num">STAGE ${node.number}</text>
      `;

      // Render RL Badge if node has RL
      if (node.hasRl) {
        svgHtml += `
          <g class="rl-badge" transform="translate(${node.x + node.w - 28}, ${node.y + 6})">
            <rect x="0" y="0" width="22" height="14" class="rl-badge-rect"/>
            <text x="11" y="10" class="rl-badge-text">RL</text>
          </g>
        `;
      }

      svgHtml += `
          <text x="${node.x + 10}" y="${node.y + 36}" class="mesh-node-title">${node.title}</text>
          <text x="${node.x + 10}" y="${node.y + 52}" class="mesh-node-sub">${node.sub}</text>
          <text x="${node.x + 10}" y="${node.y + 68}" class="mesh-node-status" style="fill: ${nodeState === 'active' ? 'var(--accent-primary)' : (nodeState === 'done' ? 'var(--accent-success)' : 'var(--text-subtle)')};">${badgeText}</text>
          <text x="${node.x + node.w - 18}" y="${node.y + 68}" class="mesh-node-info">[i]</text>
        </g>
      `;
    });

    // Render Compact Legend Overlay with 540px width
    svgHtml += `
      <g class="mesh-legend" transform="translate(40, 15)">
        <rect x="0" y="0" width="540" height="26" fill="var(--bg-panel)" stroke="var(--border-color)" rx="4" opacity="0.95"/>
        <!-- State Indicators -->
        <rect x="10" y="8" width="10" height="10" fill="var(--bg-panel)" stroke="var(--border-color)" rx="2"/>
        <text x="24" y="16" class="edge-label">Pending</text>

        <rect x="80" y="8" width="10" height="10" fill="var(--bg-panel-subtle)" stroke="var(--accent-primary)" stroke-width="2" rx="2"/>
        <text x="94" y="16" class="edge-label" style="fill: var(--accent-primary);">Active</text>

        <rect x="140" y="8" width="10" height="10" fill="var(--bg-panel-subtle)" stroke="var(--accent-success)" stroke-width="2" rx="2"/>
        <text x="154" y="16" class="edge-label" style="fill: var(--accent-success);">Done</text>

        <!-- RL Badge Legend -->
        <rect x="195" y="6" width="20" height="14" class="rl-badge-rect"/>
        <text x="205" y="16" class="rl-badge-text">RL</text>
        <text x="220" y="16" class="edge-label">Reinforcement Learning</text>

        <!-- Loop Legend -->
        <path d="M 370 13 L 388 13" stroke="#c084fc" stroke-width="2" stroke-dasharray="3 2"/>
        <text x="395" y="16" class="edge-label" style="fill: #c084fc;">↺ Retraining Loop</text>
      </g>
    `;

    meshSvg.innerHTML = svgHtml;
  }

  // Global popup handlers: RESTRICT MODAL POPUP TO EXPANDED VIEW ONLY!
  window.openSpecificModal = function(key) {
    const details = NODE_DETAILS[key];
    if (!details) return;

    // Always update status bar explanation text below mesh
    topologyExplainerText.textContent = `${details.explanation} [${details.reference}]`;

    // Show detailed popup modal ONLY IF mesh is in expanded/enlarged view mode!
    if (isMeshExpanded || topologyPanel.classList.contains("is-expanded")) {
      modalBadge.textContent = `STAGE ${details.number}`;
      modalTitle.textContent = details.title;
      modalRationale.textContent = details.rationale;
      modalPathNote.textContent = details.pathNote;
      modalDataSpec.textContent = details.dataSpec;
      modalReference.textContent = details.reference;
      infoModalOverlay.classList.remove("hidden");
    }
  };

  window.openNodeModal = function(stageIdx) {
    const node = MESH_NODES[stageIdx];
    if (node) window.openSpecificModal(node.id);
  };

  function renderWindowSlots() {
    windowSlotsContainer.innerHTML = "";
    for (let i = 0; i < 8; i++) {
      const slot = document.createElement("div");
      const obs = slidingWindow[i];
      if (obs && obs.label) {
        const isFast = obs.confidence >= 0.80;
        slot.className = `window-slot filled ${isFast ? "hit-fast" : ""}`;
        slot.innerHTML = `
          <span class="slot-num">F${i + 1}</span>
          <span class="slot-label">${obs.label.substring(0, 4)}</span>
          <span class="slot-conf">${obs.confidence.toFixed(2)}</span>
        `;
      } else {
        slot.className = "window-slot";
        slot.innerHTML = `
          <span class="slot-num">F${i + 1}</span>
          <span class="slot-label">----</span>
          <span class="slot-conf">0.00</span>
        `;
      }
      windowSlotsContainer.appendChild(slot);
    }
  }

  function syncSpeed(newVal) {
    speedMultiplier = parseFloat(newVal);
    const text = `${speedMultiplier.toFixed(1)}x`;
    if (speedSlider) speedSlider.value = speedMultiplier;
    if (speedVal) speedVal.textContent = text;
    if (panel3SpeedSlider) panel3SpeedSlider.value = speedMultiplier;
    if (panel3SpeedVal) panel3SpeedVal.textContent = text;
  }

  function setupEventListeners() {
    // Top Main Controls
    btnPlayPause.addEventListener("click", togglePlayPause);
    btnPrev.addEventListener("click", prevStep);
    btnNext.addEventListener("click", nextStep);
    btnRestart.addEventListener("click", restartCycle);

    // Panel 3 Header / Expanded Overlay Controls
    if (btnPanel3PlayPause) btnPanel3PlayPause.addEventListener("click", togglePlayPause);
    if (btnPanel3Prev) btnPanel3Prev.addEventListener("click", prevStep);
    if (btnPanel3Next) btnPanel3Next.addEventListener("click", nextStep);
    if (btnPanel3Restart) btnPanel3Restart.addEventListener("click", restartCycle);

    // Synchronized Speed Controls (0.1x Slow Motion to 4.0x Rapid)
    speedSlider.addEventListener("input", (e) => syncSpeed(e.target.value));
    if (panel3SpeedSlider) panel3SpeedSlider.addEventListener("input", (e) => syncSpeed(e.target.value));

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

    // Expand / Maximize Button Handler for Panel 3
    btnExpandMesh.addEventListener("click", toggleExpandMesh);

    btnModalClose.addEventListener("click", closeInfoModal);
    infoModalOverlay.addEventListener("click", (e) => {
      if (e.target === infoModalOverlay) closeInfoModal();
    });

    document.addEventListener("keydown", (e) => {
      if (e.key === "Escape") {
        if (!infoModalOverlay.classList.contains("hidden")) {
          closeInfoModal();
        } else if (isMeshExpanded) {
          toggleExpandMesh();
        }
      }
    });
  }

  function toggleExpandMesh() {
    isMeshExpanded = !isMeshExpanded;
    if (isMeshExpanded) {
      topologyPanel.classList.add("is-expanded");
      btnExpandMesh.title = "Minimize View (Esc)";
      expandSvg.innerHTML = `
        <polyline points="4 14 10 14 10 20"></polyline>
        <polyline points="20 10 14 10 14 4"></polyline>
        <line x1="14" y1="10" x2="21" y2="3"></line>
        <line x1="10" y1="14" x2="3" y2="21"></line>
      `;
    } else {
      topologyPanel.classList.remove("is-expanded");
      closeInfoModal(); // Close modal if open when minimizing
      btnExpandMesh.title = "Expand Panel 3 (Full Screen Overlay)";
      expandSvg.innerHTML = `
        <polyline points="15 3 21 3 21 9"></polyline>
        <polyline points="9 21 3 21 3 15"></polyline>
        <line x1="21" y1="3" x2="14" y2="10"></line>
        <line x1="3" y1="21" x2="10" y2="14"></line>
      `;
    }
    renderMeshSvg();
  }

  function closeInfoModal() {
    infoModalOverlay.classList.add("hidden");
  }

  function applyTheme(theme) {
    document.documentElement.setAttribute("data-theme", theme);
    themeToggle.checked = (theme === "light");
    themeLabelText.textContent = (theme === "light") ? "DARK MODE" : "LIGHT MODE";
    localStorage.setItem("theme", theme);
    renderCameraSvg();
    renderArmSvg();
    renderMeshSvg();
  }

  // ====== Playback Control ======
  function togglePlayPause() {
    isPlaying = !isPlaying;
    if (isPlaying) {
      const pauseIcon = `<rect x="5" y="4" width="4" height="16"></rect><rect x="15" y="4" width="4" height="16"></rect>`;
      playSvg.innerHTML = pauseIcon;
      playText.textContent = "PAUSE";
      if (panel3PlaySvg) panel3PlaySvg.innerHTML = pauseIcon;
      if (panel3PlayText) panel3PlayText.textContent = "PAUSE";
      runLoop();
    } else {
      const playIcon = `<polygon points="5,3 19,12 5,21"></polygon>`;
      playSvg.innerHTML = playIcon;
      playText.textContent = "PLAY";
      if (panel3PlaySvg) panel3PlaySvg.innerHTML = playIcon;
      if (panel3PlayText) panel3PlayText.textContent = "PLAY";
      clearTimeout(simulationTimer);
    }
  }

  function runLoop() {
    if (!isPlaying) return;
    // Stage duration formula: ranges from 300ms (at 4.0x) to 12,000ms (at 0.1x ultra-slow motion for smooth narration)
    const stageDuration = Math.max(300, 1200 / speedMultiplier);
    simulationTimer = setTimeout(() => {
      advanceStage();
      runLoop();
    }, stageDuration);
  }

  function advanceStage() {
    const isFast = isCurrentItemFastPath();

    if (currentStageIndex === 1 && isFast) {
      // Direct jump over Specialist Agents (Stage 2) straight to Arbitration Agent (Stage 3)
      currentStageIndex = 3;
    } else if (currentStageIndex < MESH_NODES.length - 1) {
      currentStageIndex++;
    } else {
      pickNextItem();
      currentStageIndex = 0;
    }
    executeCurrentStage();
  }

  function nextStep() {
    if (isPlaying) togglePlayPause();
    advanceStage();
  }

  function prevStep() {
    if (isPlaying) togglePlayPause();
    const isFast = isCurrentItemFastPath();
    if (currentStageIndex === 3 && isFast) {
      currentStageIndex = 1;
    } else if (currentStageIndex > 0) {
      currentStageIndex--;
    } else {
      currentStageIndex = 0;
    }
    executeCurrentStage();
  }

  function restartCycle() {
    if (isPlaying) togglePlayPause();
    currentStageIndex = 0;
    resetSlidingWindow();
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
    itemInArena.attachedToGripper = false;
    itemInArena.droppedInBin = false;
  }

  // ====== Execute Pipeline Stage ======
  function executeCurrentStage() {
    const node = MESH_NODES[currentStageIndex];
    const mat = MATERIALS[currentMaterialKey];
    const pose = calculatePose(mat.box);
    const isFast = isCurrentItemFastPath();

    if (node.id !== "MOTION_AGENT") {
      ledIndicator.classList.remove("led-on");
      ledIndicator.textContent = "PIN 13 LED: OFF";
    }

    switch (node.id) {
      case "COARSE_DETECTOR":
        detectionBox.classList.remove("hidden");
        const b0 = mat.box;
        detectionBox.style.left = `${b0.xmin * 100}%`;
        detectionBox.style.top = `${b0.ymin * 100}%`;
        detectionBox.style.width = `${(b0.xmax - b0.xmin) * 100}%`;
        detectionBox.style.height = `${(b0.ymax - b0.ymin) * 100}%`;
        boxLabel.textContent = `${mat.name.toUpperCase()} [CONF: ${mat.confidence.toFixed(2)}]`;
        
        slidingWindow = [{ label: mat.name, confidence: mat.confidence, box: mat.box }];
        renderWindowSlots();
        policyOutcomeText.textContent = `COARSE DETECTOR: Frame captured -> Candidate ${mat.name.toUpperCase()} (Conf: ${mat.confidence.toFixed(2)})`;
        break;

      case "CONFIDENCE_GATE":
        slidingWindow = [
          { label: mat.name, confidence: mat.confidence, box: mat.box },
          { label: mat.name, confidence: mat.confidence, box: mat.box }
        ];
        renderWindowSlots();

        if (isFast) {
          policyOutcomeText.textContent = `CONFIDENCE GATE: Conf ${mat.confidence.toFixed(2)} ≥ 0.80 -> FAST PATH HIT (Bypassing Specialist Agents)`;
        } else {
          policyOutcomeText.textContent = `CONFIDENCE GATE: Conf ${mat.confidence.toFixed(2)} < 0.80 -> AMBIGUOUS PATH (Routing to Specialist Agents 3a & 3b)`;
        }
        break;

      case "SPECIALIST_AGENTS":
        // Invoked on ambiguous path
        slidingWindow = [
          { label: mat.name, confidence: mat.confidence, box: mat.box },
          { label: mat.name, confidence: mat.confidence, box: mat.box },
          { label: mat.name, confidence: mat.confidence + 0.15, box: mat.box }
        ];
        renderWindowSlots();
        policyOutcomeText.textContent = `SPECIALIST AGENTS EVALUATION: Disambiguated ${mat.name.toUpperCase()} -> Specular/Moisture score updated (Conf: ${(mat.confidence + 0.15).toFixed(2)})`;
        break;

      case "ARBITRATION_AGENT":
        slidingWindow = [
          { label: mat.name, confidence: mat.confidence, box: mat.box },
          { label: mat.name, confidence: mat.confidence, box: mat.box },
          { label: mat.name, confidence: mat.confidence, box: mat.box }
        ];
        renderWindowSlots();

        poseAngleText.textContent = `BASE ANGLE: ${pose.baseAngle}°`;
        poseZoneText.textContent = `DIST ZONE: ${pose.zone}`;
        poseRadiusText.textContent = `RADIUS: ${pose.radius.toFixed(2)}`;
        policyOutcomeText.textContent = `ARBITRATION AGENT [RL]: Committed ${mat.name.toUpperCase()} -> Pose computed: Base ${pose.baseAngle}°, Distance Zone ${pose.zone}`;
        break;

      case "MOTION_AGENT":
        const packetStr = `${mat.code} ${mat.code} ${pose.zone} ${pose.baseAngle}\n`;
        packetTag.textContent = `PACKET: "${packetStr.trim()}"`;
        ledIndicator.classList.add("led-on");
        ledIndicator.textContent = `PIN 13 LED: FLASH (${mat.code}X)`;

        logSerial(`[PI → ARDUINO] PACKET SENT: "${packetStr.replace('\n', '\\n')}" (numBlink=${mat.code}, material=${mat.code} [${mat.name.toUpperCase()}], distance=${pose.zone}, angle=${pose.baseAngle}°)`, "tx");
        logSerial(`[ARDUINO] Parsed Serial: numBlink=${mat.code}, material=${mat.code}, distance=${pose.zone}, angle=${pose.baseAngle}. Flashing LED ${mat.code}x.`, "rx");

        const zTargets = ZONE_TARGETS[pose.zone];
        servoAngles = {
          base: pose.baseAngle,
          shoulder: zTargets.shoulder,
          elbow: zTargets.elbow,
          wrist1: 90,
          wrist2: zTargets.wrist2,
          hand: 45 // GRIP CLOSED
        };
        itemInArena.attachedToGripper = true;
        logSerial(`[ARDUINO] Executing pickUp(zone=${pose.zone}, baseAngle=${pose.baseAngle}°). Non-blocking servo step (30ms). [TIMING] pickUp-reach ms=2100`, "rx");
        break;

      case "VERIFY_AGENT":
        servoAngles = {
          base: mat.binAngle,
          shoulder: 90,
          elbow: 0,
          wrist1: 90,
          wrist2: 90,
          hand: 160 // RELEASE OPEN
        };
        itemInArena.attachedToGripper = false;
        itemInArena.droppedInBin = true;
        logSerial(`[ARDUINO] Executing dropOff(material=${mat.code}). Rotating Base to ${mat.name} Bin (${mat.binAngle}°). Release Gripper (160°). [TIMING] dropOff-move ms=5400`, "rx");
        logSerial(`[VERIFY AGENT] Bin placement check confirmed: ${mat.name.toUpperCase()} successfully released into Bin ${mat.binAngle}°.`, "highlight");
        break;

      case "RETRAINING_JOB":
        servoAngles = { ...HOME_TARGETS };
        logSerial(`[ARDUINO → PI] homeState() complete. TX: "Done Moving\\n"`, "highlight");
        logSerial(`[RETRAINING JOB] Consumed verification log -> Updated model weights (detect.tflite, specialist.pt). Feedback loop complete.`, "info");
        break;
    }

    updateTopologyUI();
    updateUI();
  }

  function updateTopologyUI() {
    const details = NODE_DETAILS[MESH_NODES[currentStageIndex].id];
    if (details) {
      topologyExplainerText.textContent = `${details.explanation} [${details.reference}]`;
    }
    renderMeshSvg();
  }

  function logSerial(msg, type = "info") {
    const timeStr = new Date().toLocaleTimeString();
    const line = document.createElement("div");
    line.className = `log-line ${type}`;
    line.textContent = `[${timeStr}] ${msg}`;
    serialLog.appendChild(line);
    serialLog.scrollTop = serialLog.scrollHeight;
  }

  function updateUI() {
    const mat = MATERIALS[currentMaterialKey];
    const pose = calculatePose(mat.box);

    angleBase.textContent = `${servoAngles.base}°`;
    angleShoulder.textContent = `${servoAngles.shoulder}°`;
    angleElbow.textContent = `${servoAngles.elbow}°`;
    angleWrist1.textContent = `${servoAngles.wrist1}°`;
    angleWrist2.textContent = `${servoAngles.wrist2}°`;
    angleHand.textContent = servoAngles.hand === 45 ? "CLOSED (45°)" : (servoAngles.hand === 160 ? "OPEN (160°)" : "HOME (90°)");

    poseAngleText.textContent = `BASE ANGLE: ${pose.baseAngle}°`;
    poseZoneText.textContent = `DIST ZONE: ${pose.zone}`;
    poseRadiusText.textContent = `RADIUS: ${pose.radius.toFixed(2)}`;

    renderCameraSvg();
    renderArmSvg();
  }

  // ====== Render Gripper Camera View (SVG with BBox & Radial Bands) ======
  function renderCameraSvg() {
    const mat = MATERIALS[currentMaterialKey];
    const pose = calculatePose(mat.box);

    let svgContent = `
      <rect width="300" height="300" fill="var(--canvas-bg)"/>
      
      <!-- Radial Distance Zone Bands from Pivot (150, 150) -->
      <circle cx="150" cy="150" r="37.5" fill="none" stroke="var(--border-color)" stroke-dasharray="2 2" opacity="0.6"/>
      <circle cx="150" cy="150" r="75" fill="none" stroke="var(--border-color)" stroke-dasharray="2 2" opacity="0.6"/>
      <circle cx="150" cy="150" r="112.5" fill="none" stroke="var(--border-color)" stroke-dasharray="2 2" opacity="0.6"/>
      <circle cx="150" cy="150" r="150" fill="none" stroke="var(--border-color)" stroke-dasharray="2 2" opacity="0.6"/>

      <!-- Center Crosshair Lines -->
      <line x1="0" y1="150" x2="300" y2="150" stroke="var(--border-color)" stroke-width="1" stroke-dasharray="3 3"/>
      <line x1="150" y1="0" x2="150" y2="300" stroke="var(--border-color)" stroke-width="1" stroke-dasharray="3 3"/>
    `;

    // Item shape rendering inside bounding box area
    const boxX = mat.box.xmin * 300;
    const boxY = mat.box.ymin * 300;
    const boxW = (mat.box.xmax - mat.box.xmin) * 300;
    const boxH = (mat.box.ymax - mat.box.ymin) * 300;

    let itemTitle = mat.name.toUpperCase();
    if (mat.name === "Metal") itemTitle = "METAL CAN";

    svgContent += `
      <rect x="${boxX}" y="${boxY}" width="${boxW}" height="${boxH}" fill="${mat.color}" stroke="var(--text-main)" stroke-width="2" rx="6"/>
      <!-- Label pill background for crisp non-overlapping item text -->
      <rect x="${boxX + 4}" y="${boxY + boxH/2 - 9}" width="${boxW - 8}" height="18" fill="rgba(15, 23, 42, 0.75)" rx="3"/>
      <text x="${boxX + boxW/2}" y="${boxY + boxH/2 + 4}" fill="#ffffff" font-family="sans-serif" font-weight="bold" font-size="10" text-anchor="middle">${itemTitle}</text>
    `;

    // Vector line from pivot (150, 150) to BBox center
    const cxPx = pose.centre_x * 300;
    const cyPx = pose.centre_y * 300;
    svgContent += `
      <line x1="150" y1="150" x2="${cxPx}" y2="${cyPx}" stroke="var(--accent-primary)" stroke-width="1.5" stroke-dasharray="2 2"/>
      <circle cx="${cxPx}" cy="${cyPx}" r="4" fill="var(--accent-danger)"/>
    `;

    cameraSvg.innerHTML = svgContent;
  }

  // ====== Render Robotic Arm Workspace Arena (SVG) ======
  function renderArmSvg() {
    const mat = MATERIALS[currentMaterialKey];
    const pose = calculatePose(mat.box);
    
    const centerX = 280;
    const centerY = 220;
    const binRadius = 145;

    let svgContent = `
      <rect width="740" height="440" fill="var(--canvas-bg)"/>
      <circle cx="${centerX}" cy="${centerY}" r="170" fill="none" stroke="var(--border-color)" stroke-dasharray="3 3"/>
      <circle cx="${centerX}" cy="${centerY}" r="115" fill="none" stroke="var(--border-color)" stroke-dasharray="3 3"/>
    `;

    // 1. Render Input Intake Tray holding mixed items
    svgContent += `
      <g transform="translate(545, 160)">
        <rect x="0" y="0" width="150" height="120" rx="6" fill="var(--bg-panel)" stroke="var(--border-color)" stroke-width="2"/>
        <text x="75" y="-10" fill="var(--text-muted)" font-size="11" font-weight="bold" font-family="sans-serif" text-anchor="middle">INPUT INTAKE TRAY</text>
        <rect x="15" y="20" width="32" height="32" fill="var(--mat-cardboard)" opacity="0.65" rx="3"/>
        <rect x="60" y="22" width="28" height="42" fill="var(--mat-glass)" opacity="0.65" rx="3"/>
        <rect x="100" y="18" width="36" height="32" fill="var(--mat-metal)" opacity="0.65" rx="3"/>
        <rect x="25" y="65" width="42" height="38" fill="var(--mat-paper)" opacity="0.65" rx="3"/>
        <rect x="80" y="60" width="38" height="48" fill="var(--mat-plastic)" opacity="0.65" rx="3"/>
      </g>
    `;

    // 2. Render 5 Output Bins at Fixed Bin Angles
    MATERIAL_KEYS.forEach((key) => {
      const bMat = MATERIALS[key];
      const rad = (bMat.binAngle - 90) * (Math.PI / 180);
      const binX = centerX + Math.cos(rad) * binRadius;
      const binY = centerY + Math.sin(rad) * binRadius;

      svgContent += `
        <g transform="translate(${binX}, ${binY})">
          <rect x="-35" y="-20" width="70" height="40" rx="4" fill="var(--bg-panel)" stroke="${bMat.color}" stroke-width="2"/>
          <text x="0" y="-3" fill="var(--text-main)" font-size="10" font-weight="bold" text-anchor="middle">${bMat.name.toUpperCase()}</text>
          <text x="0" y="9" fill="${bMat.color}" font-size="9" font-family="monospace" text-anchor="middle">${bMat.binAngle}° BIN</text>
        </g>
      `;
    });

    // 3. Render Item Position (Intake tray center vs Picked gripper vs Dropped bin)
    let itemX = 620;
    let itemY = 220;

    if (itemInArena.droppedInBin) {
      const rad = (mat.binAngle - 90) * (Math.PI / 180);
      itemX = centerX + Math.cos(rad) * binRadius;
      itemY = centerY + Math.sin(rad) * binRadius;
    } else if (itemInArena.attachedToGripper) {
      const armRad = (servoAngles.base - 90) * (Math.PI / 180);
      itemX = centerX + Math.cos(armRad) * 125;
      itemY = centerY + Math.sin(armRad) * 125;
    }

    svgContent += `
      <g transform="translate(${itemX}, ${itemY})">
        <circle r="13" fill="${mat.color}" stroke="#ffffff" stroke-width="2"/>
      </g>
    `;

    // 4. Render 6-DOF Robotic Arm Joint Kinematics
    const baseRad = (servoAngles.base - 90) * (Math.PI / 180);
    const armLength1 = 70;
    const armLength2 = 55;

    const elbowX = centerX + Math.cos(baseRad) * armLength1;
    const elbowY = centerY + Math.sin(baseRad) * armLength1;

    const gripperX = elbowX + Math.cos(baseRad) * armLength2;
    const gripperY = elbowY + Math.sin(baseRad) * armLength2;

    svgContent += `
      <!-- Base Turret Title Label -->
      <text x="${centerX}" y="${centerY - 28}" fill="var(--text-muted)" font-size="9" font-weight="bold" font-family="sans-serif" text-anchor="middle">BASE TURRET</text>

      <!-- Base Turret Circle -->
      <circle cx="${centerX}" cy="${centerY}" r="22" fill="var(--bg-panel)" stroke="var(--accent-primary)" stroke-width="3"/>
      <circle cx="${centerX}" cy="${centerY}" r="6" fill="var(--accent-primary)"/>

      <!-- Arm Links -->
      <line x1="${centerX}" y1="${centerY}" x2="${elbowX}" y2="${elbowY}" stroke="var(--accent-primary)" stroke-width="7" stroke-linecap="round"/>
      <circle cx="${elbowX}" cy="${elbowY}" r="9" fill="var(--bg-panel)" stroke="var(--text-main)" stroke-width="2"/>
      <line x1="${elbowX}" y1="${elbowY}" x2="${gripperX}" y2="${gripperY}" stroke="var(--text-muted)" stroke-width="5" stroke-linecap="round"/>

      <!-- Gripper Hand -->
      <g transform="translate(${gripperX}, ${gripperY}) rotate(${servoAngles.base})">
        <path d="M-8,-5 L0,0 L-8,5" fill="none" stroke="${servoAngles.hand === 45 ? 'var(--accent-danger)' : 'var(--accent-success)'}" stroke-width="3" stroke-linecap="round"/>
        <path d="M8,-5 L0,0 L8,5" fill="none" stroke="${servoAngles.hand === 45 ? 'var(--accent-danger)' : 'var(--accent-success)'}" stroke-width="3" stroke-linecap="round"/>
        <circle cx="0" cy="0" r="4" fill="var(--text-main)"/>
      </g>
    `;

    // 5. Render Arduino Microcontroller Board
    svgContent += `
      <g transform="translate(20, 350)">
        <rect width="135" height="70" rx="5" fill="var(--bg-panel)" stroke="var(--border-color)" stroke-width="2"/>
        <text x="67" y="20" fill="var(--text-main)" font-family="monospace" font-weight="bold" font-size="10" text-anchor="middle">ARDUINO UNO</text>
        <circle cx="22" cy="45" r="7" fill="${ledIndicator.classList.contains('led-on') ? 'var(--accent-danger)' : 'var(--border-color)'}" stroke="var(--text-main)" stroke-width="1"/>
        <text x="36" y="49" fill="var(--text-muted)" font-size="9" font-family="monospace">PIN 13 LED</text>
      </g>
    `;

    armSvg.innerHTML = svgContent;
  }

  // Start Engine
  init();
});
