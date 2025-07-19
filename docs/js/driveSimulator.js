// Game state
let robot = {
    x: 400,
    y: 200,
    angle: 0,
    vx: 0,
    vy: 0,
    angularVelocity: 0
};

let inputs = {
    forward: 0,
    turn: 0,
    strafe: 0,
    rotation: 0
};

let keys = {};
let driveType = 'tank';

// Canvas setup
const canvas = document.getElementById('gameField');
const ctx = canvas.getContext('2d');

// Input handling
document.addEventListener('keydown', (e) => {
    keys[e.key.toLowerCase()] = true;
    e.preventDefault();
});

document.addEventListener('keyup', (e) => {
    keys[e.key.toLowerCase()] = false;
    e.preventDefault();
});

function processInputs() {
    // Reset inputs
    inputs.forward = 0;
    inputs.turn = 0;
    inputs.strafe = 0;
    inputs.rotation = 0;

    // WASD and Arrow keys for primary movement
    if (keys['w'] || keys['arrowup']) inputs.forward += 1;
    if (keys['s'] || keys['arrowdown']) inputs.forward -= 1;
    if (keys['a'] || keys['arrowleft']) inputs.turn -= 1;
    if (keys['d'] || keys['arrowright']) inputs.turn += 1;

    // Swerve-specific controls
    if (driveType === 'swerve') {
        if (keys['q']) inputs.rotation -= 1;
        if (keys['e']) inputs.rotation += 1;
        if (keys['z']) inputs.strafe -= 1;
        if (keys['c']) inputs.strafe += 1;
    }

    // Clamp inputs
    inputs.forward = Math.max(-1, Math.min(1, inputs.forward));
    inputs.turn = Math.max(-1, Math.min(1, inputs.turn));
    inputs.strafe = Math.max(-1, Math.min(1, inputs.strafe));
    inputs.rotation = Math.max(-1, Math.min(1, inputs.rotation));
}

function updateRobot(dt) {
    if (driveType === 'tank') {
        updateTankDrive(dt);
    } else {
        updateSwerveDrive(dt);
    }

    // Apply physics
    robot.x += robot.vx * dt;
    robot.y += robot.vy * dt;
    robot.angle += robot.angularVelocity * dt;

    // Keep robot on field
    robot.x = Math.max(25, Math.min(canvas.width - 25, robot.x));
    robot.y = Math.max(25, Math.min(canvas.height - 25, robot.y));

    // Normalize angle
    while (robot.angle > Math.PI) robot.angle -= 2 * Math.PI;
    while (robot.angle < -Math.PI) robot.angle += 2 * Math.PI;

    // Apply damping
    robot.vx *= 0.85;
    robot.vy *= 0.85;
    robot.angularVelocity *= 0.8;
}

function updateTankDrive(dt) {
    const maxSpeed = 200; // pixels per second
    const maxTurnRate = 3; // radians per second

    // Tank drive: differential steering
    const leftPower = inputs.forward + inputs.turn;
    const rightPower = inputs.forward - inputs.turn;

    // Calculate forward velocity based on average of both sides
    const forwardVel = (leftPower + rightPower) / 2 * maxSpeed;
    
    // Calculate turn rate based on difference
    const turnRate = (rightPower - leftPower) / 2 * maxTurnRate;

    // Convert to robot-relative movement
    robot.vx = Math.cos(robot.angle) * forwardVel;
    robot.vy = Math.sin(robot.angle) * forwardVel;
    robot.angularVelocity = turnRate;
}

function updateSwerveDrive(dt) {
    const maxSpeed = 250; // pixels per second
    const maxTurnRate = 4; // radians per second

    // Swerve drive: field-relative movement
    const fieldVx = inputs.strafe * maxSpeed + inputs.turn * maxSpeed;
    const fieldVy = -inputs.forward * maxSpeed; // Negative because canvas Y is flipped

    // Convert field-relative to robot-relative
    robot.vx = fieldVx * Math.cos(-robot.angle) - fieldVy * Math.sin(-robot.angle);
    robot.vy = fieldVx * Math.sin(-robot.angle) + fieldVy * Math.cos(-robot.angle);
    
    // Independent rotation
    robot.angularVelocity = inputs.rotation * maxTurnRate;
}

function drawField() {
    // Clear canvas
    ctx.fillStyle = '#2d5a27';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw field pattern
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.lineWidth = 1;
    
    // Grid lines
    for (let x = 0; x <= canvas.width; x += 40) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, canvas.height);
        ctx.stroke();
    }
    
    for (let y = 0; y <= canvas.height; y += 40) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(canvas.width, y);
        ctx.stroke();
    }

    // Center line
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(canvas.width / 2, 0);
    ctx.lineTo(canvas.width / 2, canvas.height);
    ctx.stroke();
}

function drawRobot() {
    ctx.save();
    ctx.translate(robot.x, robot.y);
    ctx.rotate(robot.angle);

    // Robot body
    ctx.fillStyle = '#ff6b35';
    ctx.fillRect(-20, -15, 40, 30);
    
    // Robot outline
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.strokeRect(-20, -15, 40, 30);

    // Direction indicator
    ctx.fillStyle = '#00ff88';
    ctx.beginPath();
    ctx.moveTo(15, 0);
    ctx.lineTo(25, -8);
    ctx.lineTo(25, 8);
    ctx.closePath();
    ctx.fill();

    // Wheels/modules based on drive type
    if (driveType === 'tank') {
        drawTankWheels();
    } else {
        drawSwerveModules();
    }

    ctx.restore();
}

function drawTankWheels() {
    ctx.fillStyle = '#333';
    // Left side wheels
    ctx.fillRect(-25, -12, 8, 8);
    ctx.fillRect(-25, 4, 8, 8);
    // Right side wheels
    ctx.fillRect(17, -12, 8, 8);
    ctx.fillRect(17, 4, 8, 8);
}

function drawSwerveModules() {
    const modulePositions = [
        [-15, -10], [15, -10],
        [-15, 10], [15, 10]
    ];

    ctx.fillStyle = '#4a90e2';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1;

    modulePositions.forEach(([x, y]) => {
        ctx.save();
        ctx.translate(x, y);
        
        // Module body
        ctx.fillRect(-4, -4, 8, 8);
        ctx.strokeRect(-4, -4, 8, 8);
        
        // Wheel direction indicator
        ctx.strokeStyle = '#00ff88';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-6, 0);
        ctx.lineTo(6, 0);
        ctx.stroke();
        
        ctx.restore();
    });
}

function updateUI() {
    // Update input bars
    updateInputBar('forwardBar', inputs.forward);
    updateInputBar('turnBar', inputs.turn);
    updateInputBar('strafeBar', inputs.strafe);
    updateInputBar('rotationBar', inputs.rotation);

    // Update telemetry
    document.getElementById('posX').textContent = (robot.x / 40).toFixed(1);
    document.getElementById('posY').textContent = ((canvas.height - robot.y) / 40).toFixed(1);
    document.getElementById('angle').textContent = (robot.angle * 180 / Math.PI).toFixed(1);
    
    const speed = Math.sqrt(robot.vx * robot.vx + robot.vy * robot.vy) / 40;
    document.getElementById('speed').textContent = speed.toFixed(1);
}

function updateInputBar(id, value) {
    const bar = document.getElementById(id);
    const width = Math.abs(value) * 50; // 50% max width
    const left = value >= 0 ? 50 : 50 - width;
    
    bar.style.width = width + '%';
    bar.style.left = left + '%';
    
    // Color coding
    if (Math.abs(value) < 0.1) {
        bar.style.background = '#444';
    } else if (value > 0) {
        bar.style.background = 'linear-gradient(90deg, #44ff44, #44aaff)';
    } else {
        bar.style.background = 'linear-gradient(90deg, #ff4444, #ffaa44)';
    }
}

function setDriveType(type) {
    driveType = type;
    
    // Update UI
    document.querySelectorAll('.drive-type-btn').forEach(btn => {
        btn.classList.remove('active');
    });
    
    event.target.classList.add('active');
    
    // Show/hide swerve inputs
    const swerveInputs = document.getElementById('swerveInputs');
    swerveInputs.style.display = type === 'swerve' ? 'block' : 'none';
    
    // Reset robot state
    resetRobot();
}

function resetRobot() {
    robot.x = canvas.width / 2;
    robot.y = canvas.height / 2;
    robot.angle = 0;
    robot.vx = 0;
    robot.vy = 0;
    robot.angularVelocity = 0;
    
    inputs.forward = 0;
    inputs.turn = 0;
    inputs.strafe = 0;
    inputs.rotation = 0;
}

// Game loop
let lastTime = 0;
function gameLoop(timestamp) {
    const dt = (timestamp - lastTime) / 1000;
    lastTime = timestamp;

    if (dt < 0.1) { // Prevent large jumps
        processInputs();
        updateRobot(dt);
        drawField();
        drawRobot();
        updateUI();
    }

    requestAnimationFrame(gameLoop);
}

// Initialize
resetRobot();
requestAnimationFrame(gameLoop);

// Handle canvas resize
function resizeCanvas() {
    const container = canvas.parentElement;
    const rect = container.getBoundingClientRect();
    const aspectRatio = 2; // 2:1 aspect ratio
    
    let width = Math.min(rect.width - 20, 800);
    let height = width / aspectRatio;
    
    if (height > 400) {
        height = 400;
        width = height * aspectRatio;
    }
    
    canvas.width = width;
    canvas.height = height;
    
    // Reset robot to center
    robot.x = canvas.width / 2;
    robot.y = canvas.height / 2;
}

window.addEventListener('resize', resizeCanvas);
resizeCanvas();