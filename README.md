# AI Waste Sorting Robotic Arm — Interactive Web Simulation

An interactive, single-page engineering presentation simulation of an AI-powered Waste Sorting Robotic Arm system grounded in a real TensorFlow Lite vision engine (`detect_script.py`) and an Arduino 6-DOF servo controller (`newprogramwaste.ino`).

## 🚀 Live Demo & Hosting
Designed to be hosted instantly as a zero-dependency static web application on **Vercel** or **GitHub Pages**.

## 🛠️ Features
- **Visual Vision Pipeline**: 300x300 TFLite input tensor simulation with confidence scores & 5-frame consecutive debouncing filter.
- **6-DOF Robotic Arm Arena**: Real-time SVG rendering of joint angle kinematics (Base, Shoulder, Elbow, Wrist 1, Wrist 2, Hand Gripper) with full pick-and-place movement across 5 material bins (Cardboard 0°, Glass 45°, Metal 90°, Paper 135°, Plastic 180°).
- **Serial Terminal Protocol**: Live streaming of raw packet transmissions (`COM7` @ `9600 Baud`).
- **Academic Presentation Controls**: Play/Pause, Step Next/Prev, Speed Multiplier (0.5x–4x), Next Item Override, Show Source Toggle, and full Dual Light/Dark Theme toggle.

## 📦 Project Structure
```
.
├── index.html   # Main Dashboard HTML
├── styles.css   # Industrial Dashboard Styles & Dual-Theme Tokens
├── app.js       # Simulation Engine & 6-DOF Kinematics Renderer
└── vercel.json  # Vercel Deployment Configuration
```
