body {
    margin: 0;
    padding: 20px;
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
    color: white;
    min-height: 100vh;
}

.driveSimulator-container {
    max-width: 1200px;
    margin: 0 auto;
    background: rgba(255, 255, 255, 0.1);
    border-radius: 20px;
    backdrop-filter: blur(10px);
    border: 1px solid rgba(255, 255, 255, 0.2);
    padding: 30px;
    box-shadow: 0 20px 40px rgba(0, 0, 0, 0.3);
}

.title {
    text-align: center;
    font-size: 2.5em;
    margin-bottom: 30px;
    background: linear-gradient(45deg, #00ff88, #00ccff);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
    background-clip: text;
    font-weight: bold;
    text-shadow: 0 0 20px rgba(0, 255, 136, 0.3);
}

.controls-section {
    display: flex;
    gap: 30px;
    margin-bottom: 30px;
    flex-wrap: wrap;
}

.control-panel {
    background: rgba(0, 0, 0, 0.3);
    padding: 20px;
    border-radius: 15px;
    border: 1px solid rgba(255, 255, 255, 0.1);
    flex: 1;
    min-width: 280px;
}

.control-group {
    margin-bottom: 20px;
}

.control-group h3 {
    margin: 0 0 15px 0;
    color: #00ff88;
    font-size: 1.2em;
}

.drive-type-selector {
    display: flex;
    gap: 15px;
    margin-bottom: 20px;
}

.drive-type-btn {
    flex: 1;
    padding: 12px 20px;
    border: none;
    border-radius: 10px;
    background: rgba(255, 255, 255, 0.1);
    color: white;
    cursor: pointer;
    transition: all 0.3s ease;
    font-weight: bold;
}

.drive-type-btn:hover {
    background: rgba(255, 255, 255, 0.2);
    transform: translateY(-2px);
}

.drive-type-btn.active {
    background: linear-gradient(45deg, #00ff88, #00ccff);
    color: #000;
    box-shadow: 0 5px 15px rgba(0, 255, 136, 0.3);
}

.field-container {
    position: relative;
    background: #2d5a27;
    border: 4px solid #fff;
    border-radius: 15px;
    margin: 0 auto;
    overflow: hidden;
    box-shadow: 0 10px 30px rgba(0, 0, 0, 0.5);
}

#gameField {
    display: block;
    background: linear-gradient(45deg, #2d5a27 25%, #245a1f 25%, #245a1f 50%, #2d5a27 50%, #2d5a27 75%, #245a1f 75%, #245a1f);
    background-size: 40px 40px;
}

.input-display {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 15px;
    margin-top: 15px;
}

.input-group {
    background: rgba(0, 0, 0, 0.2);
    padding: 15px;
    border-radius: 10px;
    border: 1px solid rgba(255, 255, 255, 0.1);
}

.input-group h4 {
    margin: 0 0 10px 0;
    color: #00ccff;
    font-size: 1em;
}

.input-bar {
    background: rgba(0, 0, 0, 0.3);
    height: 20px;
    border-radius: 10px;
    margin: 5px 0;
    overflow: hidden;
    position: relative;
}

.input-bar-fill {
    height: 100%;
    background: linear-gradient(90deg, #ff4444, #ffaa44, #44ff44, #44aaff, #4444ff);
    border-radius: 10px;
    transition: width 0.1s ease;
    position: absolute;
    left: 50%;
    transform: translateX(-50%);
}

.input-label {
    font-size: 0.9em;
    color: rgba(255, 255, 255, 0.8);
    margin-bottom: 5px;
}

.instructions {
    background: rgba(0, 0, 0, 0.2);
    padding: 20px;
    border-radius: 15px;
    margin-top: 20px;
    border: 1px solid rgba(255, 255, 255, 0.1);
}

.instructions h3 {
    color: #00ff88;
    margin-top: 0;
}

.key-group {
    display: inline-block;
    margin: 5px 10px 5px 0;
}

.key {
    background: rgba(255, 255, 255, 0.1);
    padding: 5px 10px;
    border-radius: 5px;
    border: 1px solid rgba(255, 255, 255, 0.2);
    font-family: monospace;
    font-weight: bold;
    margin-right: 5px;
}

.robot-info {
    position: absolute;
    top: 10px;
    left: 10px;
    background: rgba(0, 0, 0, 0.7);
    padding: 10px;
    border-radius: 10px;
    color: white;
    font-size: 0.9em;
    min-width: 150px;
}

.reset-btn {
    padding: 12px 25px;
    background: linear-gradient(45deg, #ff6b6b, #ff8e8e);
    border: none;
    border-radius: 10px;
    color: white;
    font-weight: bold;
    cursor: pointer;
    transition: all 0.3s ease;
    margin-top: 15px;
}

.reset-btn:hover {
    transform: translateY(-2px);
    box-shadow: 0 5px 15px rgba(255, 107, 107, 0.4);
}

.telemetry {
    font-family: 'Courier New', monospace;
    font-size: 0.85em;
    line-height: 1.2;
}

@media (max-width: 768px) {
    .controls-section {
        flex-direction: column;
    }
    
    .input-display {
        grid-template-columns: 1fr;
    }
}