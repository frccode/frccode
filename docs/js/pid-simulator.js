// File: docs/Learn/Chapter_4/js/distance-control-simulator.js

// Namespace for distance control simulator to avoid conflicts
window.DistanceControlSim = {
    robot: null,
    robotPosition: 0,
    targetPosition: 50,
    currentSpeed: 0,
    isMoving: false,
    simulationInterval: null,
    targetReached: false
};

// Initialize the distance control simulator
function initializeDistanceControlSimulator() {
    DistanceControlSim.robot = document.getElementById('distance-robot');
    updateDistanceControlDisplay();
}

// Reset the distance control simulation
function resetDistanceControlSim() {
    DistanceControlSim.isMoving = false;
    DistanceControlSim.targetReached = false;
    clearInterval(DistanceControlSim.simulationInterval);
    DistanceControlSim.simulationInterval = null;
    
    DistanceControlSim.robotPosition = 0;
    DistanceControlSim.currentSpeed = 0;
    
    updateDistanceControlDisplay();
}

// Manual control for the simulator
function manualDistanceControl(speed) {
    DistanceControlSim.currentSpeed = speed; // Max speed is 1.0
    
    if (speed !== 0) {
        DistanceControlSim.isMoving = true;
        
        if (!DistanceControlSim.simulationInterval) {
            DistanceControlSim.simulationInterval = setInterval(() => {
                if (DistanceControlSim.isMoving) {
                    // Move the robot based on current speed
                    DistanceControlSim.robotPosition += DistanceControlSim.currentSpeed * 0.5;
                    
                    // Keep robot within bounds (0 to 50 meters)
                    DistanceControlSim.robotPosition = Math.max(0, Math.min(50, DistanceControlSim.robotPosition));
                    
                    // Check if target is reached
                    if (Math.abs(DistanceControlSim.targetPosition - DistanceControlSim.robotPosition) < 0.5) {
                        if (!DistanceControlSim.targetReached) {
                            DistanceControlSim.targetReached = true;
                            document.getElementById('distance-success-message').style.display = 'block';
                            setTimeout(() => {
                                document.getElementById('distance-success-message').style.display = 'none';
                            }, 3000);
                        }
                    } else {
                        DistanceControlSim.targetReached = false;
                        document.getElementById('distance-success-message').style.display = 'none';
                    }
                    
                    updateDistanceControlDisplay();
                }
            }, 50);
        }
    } else {
        // Stop the robot
        DistanceControlSim.isMoving = false;
        DistanceControlSim.currentSpeed = 0;
        clearInterval(DistanceControlSim.simulationInterval);
        DistanceControlSim.simulationInterval = null;
        updateDistanceControlDisplay();
    }
}

// Update the visual display
function updateDistanceControlDisplay() {
    const fieldDisplay = document.getElementById('distance-field-display');
    if (!fieldDisplay) return;
    
    const fieldWidth = fieldDisplay.offsetWidth - 90;
    const robotPixelPosition = (DistanceControlSim.robotPosition / 50) * fieldWidth + 20; // Changed to 50 meters
    
    if (DistanceControlSim.robot) {
        DistanceControlSim.robot.style.left = robotPixelPosition + 'px';
    }
    
    const distanceToTarget = DistanceControlSim.targetPosition - DistanceControlSim.robotPosition;
    const status = DistanceControlSim.targetReached ? 'Target Reached!' : 
                  DistanceControlSim.isMoving ? 'Moving' : 'Stopped';
    
    const statusDisplay = document.getElementById('distance-status-display');
    if (statusDisplay) {
        statusDisplay.innerHTML = `
            Position: ${DistanceControlSim.robotPosition.toFixed(1)}m<br>
            Distance to Target: ${distanceToTarget.toFixed(1)}m<br>
            Speed: ${DistanceControlSim.currentSpeed.toFixed(1)}<br>
            Status: ${status}
        `;
    }
    
    // Change robot color based on distance
    if (DistanceControlSim.robot) {
        if (distanceToTarget < 2.5) {
            DistanceControlSim.robot.style.background = '#28a745'; // Green when close
        } else if (distanceToTarget < 10) {
            DistanceControlSim.robot.style.background = '#ffc107'; // Yellow when getting close
        } else {
            DistanceControlSim.robot.style.background = '#007bff'; // Blue when far
        }
    }
}

// Initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    setTimeout(initializeDistanceControlSimulator, 100);
});