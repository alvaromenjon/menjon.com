/**
 * Simplified and interactive spacecraft/rocket attitude control simulation
 * using RCS thrusters and a PID controller.
 * 
 * @author Álvaro Menjón
 * @version 0.1.0
 */

class RocketSim {
    
    // ====================================================================
    // SIMULATION CONSTANTS
    // ====================================================================
    
    /**
     * Physics parameters.
     */
    static PHYSICS = {
        MASS: 750,
        MAX_FRAME_TIME: 1/60
    };
    
    /**
     * Spacecraft/rocket geometry parameters.
     */
    static ROCKET = {
        // Main body dimensions in pixels
        LENGTH: 160,
        WIDTH: 32,
        NOSE_CONE_LENGTH: 50,
        LEVER_ARM_RATIO: 0.4
    };
    
    /**
     * Reaction Control System (RCS) parameters.
     * These values control the spacecraft's attitude control behavior and visual effects.
     */
    static RCS = {
        // PID controller gains [Proportional, Derivative]
        GAINS: [0.632, 2.05],
        
        // Deadband: minimum error before thrusters fire (in radians)
        DEADBAND: 0.2 * Math.PI / 180,
        
        // Individual thruster force in Newtons
        THRUST_FORCE: 200,
        
        // Thruster response time constant (seconds)
        // Models valve opening/closing dynamics
        TIME_CONSTANT: 0.01,
        
        // Visual parameters for exhaust plume rendering
        NOZZLE_DIAMETER: 11,  // Reference dimension for plume scaling
        
        // Exhaust expansion angles
        CONE_ANGLE_15: Math.tan(15 * Math.PI / 180),  // Core jet angle
        CONE_ANGLE_30: Math.tan(30 * Math.PI / 180)   // Full expansion angle
    };
    
    /**
     * User interface and control parameters.
     * Settings that affect how the user interacts with the simulation.
     */
    static CONTROL = {
        // Hitbox for rocket selection
        HITBOX_PADDING_X: 1.5,
        HITBOX_PADDING_Y: 1.2,
        
        // Mouse drag sensitivity and limits
        DRAG_RATE_FACTOR: 0.5,    // How responsive dragging feels
        MAX_DRAG_RATE: 5          // Maximum spin rate from dragging (rad/s)
    };
    
    /**
     * Visual and rendering parameters.
     * Control the appearance and performance of the simulation graphics.
     */
    static VISUAL = {
        // Starfield density (stars per pixel)
        STAR_DENSITY: 0.00005,
        
        // Number of pre-rendered plume textures for smooth animation
        PLUME_CACHE_STEPS: 20,
        
        // Trail fade rate
        TRAIL_FADE: 0.25
    };

    // ====================================================================
    // CONSTRUCTOR AND INITIALIZATION
    // ====================================================================
    
    /**
     * Initialize the rocket simulation system.
     * Sets up all subsystems: physics, graphics, input handling, and game loop.
     */
    constructor() {
        // Get canvas element and rendering context
        this.canvas = document.getElementById('canvas');
        this.ctx = this.canvas.getContext('2d');
        
        // Initialize all simulation subsystems
        this.initializeState();        // Set up initial conditions
        this.setupCanvas();            // Configure display and viewport
        this.preRenderPlumes();        // Generate exhaust plume textures
        this.setupInput();             // Configure mouse/touch input
        this.startSimulation();        // Begin main simulation loop
    }

    /**
     * Initialize all simulation state variables.
     * This includes rocket position/orientation, physics state, and visual elements.
     */
    initializeState() {
        // Rocket position and orientation state
        this.rocket = {
            x: 0,                    // Screen X coordinate (set in setupCanvas)
            y: 0,                    // Screen Y coordinate (set in setupCanvas)
            angle: 0,                // Current orientation in radians (0 = pointing up)
            isDragging: false,       // Whether user is currently dragging
            dragAngleOffset: 0       // Angular offset during drag operations
        };

        // Calculate moment of inertia for rotational dynamics
        // Uses solid cylinder approximation: I = (1/12) * m * (3r² + h²)
        const radius = RocketSim.ROCKET.WIDTH / 2 / 100;
        const height = RocketSim.ROCKET.LENGTH / 100;
        
        this.physics = {
            angularVelocity: 0,      // Current rotation rate (rad/s)
            
            // Moment of inertia
            momentOfInertia: (1/12) * RocketSim.PHYSICS.MASS * (3 * radius * radius + height * height),
            
            // Previous frame values for numerical integration
            lastAngle: 0,
            lastTime: performance.now()
        };

        // Guidance system state (where the rocket should point)
        this.guidance = { 
            targetAngle: 0           // Desired orientation in radians
        };

        // Calculate maximum torque from RCS thrusters
        const leverArm = RocketSim.ROCKET.LENGTH * RocketSim.ROCKET.LEVER_ARM_RATIO / 100;
        
        this.rcs = {
            maxTorque: RocketSim.RCS.THRUST_FORCE * leverArm,
            activePulse: null        // Current thruster firing state
        };

        // Visual elements
        this.starfield = [];         // Background stars for space environment
        this.plumeCache = [];        // Pre-rendered exhaust plume textures
    }

    // ====================================================================
    // PHYSICS SIMULATION
    // ====================================================================
    
    /**
     * Main physics update loop.
     * Integrates equations of motion and applies control torques.
     * 
     * @param {number} deltaTime - Time step in seconds
     */
    updatePhysics(deltaTime) {
        // Choose physics mode based on user interaction
        if (this.rocket.isDragging) {
            // User is manually spinning the rocket
            this.updateDragPhysics(deltaTime);
        } else {
            // Automatic attitude control is active
            this.updateGuidancePhysics(deltaTime);
        }

        // Apply Newton's second law for rotation: τ = I × α
        // Where: τ = torque, I = moment of inertia, α = angular acceleration
        const appliedTorque = this.getAppliedTorque();
        const angularAcceleration = appliedTorque / this.physics.momentOfInertia;
        
        // Integrate angular acceleration to get angular velocity
        // ω(t+Δt) = ω(t) + α × Δt
        this.physics.angularVelocity += angularAcceleration * deltaTime;
        
        // Integrate angular velocity to get orientation
        // θ(t+Δt) = θ(t) + ω × Δt
        this.rocket.angle += this.physics.angularVelocity * deltaTime;
        
        // Keep angle in standard range [-π, π] for consistency
        this.rocket.angle = this.normalizeAngle(this.rocket.angle);
        this.physics.lastAngle = this.rocket.angle;
    }

    /**
     * Handle physics during user drag operations.
     * Estimates angular velocity from mouse movement.
     * 
     * @param {number} deltaTime - Time step in seconds
     */
    updateDragPhysics(deltaTime) {
        // Calculate how much the angle changed since last frame
        const deltaAngle = this.normalizeAngle(this.rocket.angle - this.physics.lastAngle);
        
        if (deltaTime > 0) {
            // Estimate angular velocity from angle change
            const rawVelocity = deltaAngle / deltaTime * RocketSim.CONTROL.DRAG_RATE_FACTOR;
            
            // Clamp to reasonable limits to prevent excessive spinning
            this.physics.angularVelocity = Math.max(
                -RocketSim.CONTROL.MAX_DRAG_RATE, 
                Math.min(RocketSim.CONTROL.MAX_DRAG_RATE, rawVelocity)
            );
        }
        
        // Shut down RCS thrusters during manual control
        // User has taken manual control, so automatic thrusters should not fire
        this.updateRCS(deltaTime, 0, false);
    }

    /**
     * Automatic attitude control using PID controller.
     * This is the core of spacecraft attitude determination and control (ADCS).
     * 
     * @param {number} deltaTime - Time step in seconds
     */
    updateGuidancePhysics(deltaTime) {
        // Calculate attitude error
        const angleError = this.normalizeAngle(this.guidance.targetAngle - this.rocket.angle);
        
        // Calculate rate error
        const rateError = -this.physics.angularVelocity;
        
        // PID Controller: Proportional-Derivative control
        const requiredTorque = RocketSim.RCS.GAINS[0] * angleError + RocketSim.RCS.GAINS[1] * rateError;
        
        // Deadband prevents thruster firing for tiny errors
        const shouldFire = Math.abs(angleError) > RocketSim.RCS.DEADBAND;
        
        // Update thruster system based on control requirements
        this.updateRCS(deltaTime, requiredTorque, shouldFire);
    }

    /**
     * Update Reaction Control System (RCS) thruster state.
     *
     * @param {number} deltaTime - Time step in seconds
     * @param {number} requiredTorque - Desired torque output from controller
     * @param {boolean} shouldFire - Whether thrusters should be active
     */
    updateRCS(deltaTime, requiredTorque, shouldFire) {
        const pulse = this.rcs.activePulse;

        // Start new thruster pulse if needed
        if (!pulse && shouldFire && Math.abs(requiredTorque) > 0.1) {
            this.rcs.activePulse = {
                intensity: 0,                          // Start at zero intensity
                direction: Math.sign(requiredTorque)   // +1 for clockwise, -1 for counterclockwise
            };
        }

        // Update existing thruster pulse
        if (pulse) {
            // Determine target intensity based on control requirements
            const shouldContinue = shouldFire && Math.sign(requiredTorque) === pulse.direction;
            const targetIntensity = shouldContinue ? 1.0 : 0;
            
            // Model thruster valve dynamics with first-order response
            // This creates startup/shutdown curves instead of instant on/off
            const rateOfChange = (targetIntensity - pulse.intensity) / RocketSim.RCS.TIME_CONSTANT;
            pulse.intensity += rateOfChange * deltaTime;
            
            // Clamp intensity to valid range
            pulse.intensity = Math.max(0, Math.min(1, pulse.intensity));
            
            // Remove pulse when it's essentially off
            if (pulse.intensity < 0.001) {
                this.rcs.activePulse = null;
            }
        }
    }

    /**
     * Calculate the net torque currently being applied to the spacecraft.
     * 
     * @returns {number} Applied torque in Newton-meters
     */
    getAppliedTorque() {
        if (!this.rcs.activePulse) {
            return 0;
        }
        
        // Torque = direction × maximum torque × current intensity
        return this.rcs.activePulse.direction * this.rcs.maxTorque * this.rcs.activePulse.intensity;
    }

    /**
     * Normalize angle to the range [-π, π].
     *
     * @param {number} angle - Input angle in radians
     * @returns {number} Normalized angle in range [-π, π]
     */
    normalizeAngle(angle) {
        const twoPI = 2 * Math.PI;
        return ((angle + Math.PI) % twoPI + twoPI) % twoPI - Math.PI;
    }

    // ====================================================================
    // EXHAUST PLUME GENERATION AND RENDERING
    // ====================================================================
    
    /**
     * Pre-generate exhaust plume textures for smooth animation.
     * This avoids expensive pixel calculations during real-time rendering.
     */
    preRenderPlumes() {
        // Generate textures for different thruster intensities
        for (let i = 0; i <= RocketSim.VISUAL.PLUME_CACHE_STEPS; i++) {
            const intensity = i / RocketSim.VISUAL.PLUME_CACHE_STEPS;
            this.plumeCache.push(this.generatePlumeTexture(intensity));
        }
    }

    /**
     * Generate a single exhaust plume texture.
     * Models the expansion of hot exhaust gases from a rocket nozzle.
     * 
     * @param {number} intensity - Thruster intensity (0.0 to 1.0)
     * @returns {HTMLCanvasElement} Pre-rendered plume texture
     */
    generatePlumeTexture(intensity) {
        const D = RocketSim.RCS.NOZZLE_DIAMETER;  // Reference diameter
        
        // Calculate plume dimensions based on nozzle theory
        // Length: exhaust expands roughly 40 diameters before dispersing
        const haloLength = 40 * D;
        
        // Width: 30° expansion angle for fully developed supersonic jet
        const haloWidth = haloLength * RocketSim.RCS.CONE_ANGLE_30;

        // Create off-screen canvas for texture generation
        const canvas = document.createElement('canvas');
        canvas.width = haloLength + D;
        canvas.height = haloWidth * 2;
        
        const ctx = canvas.getContext('2d');
        const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        
        // Generate plume pixels using physics-based model
        this.renderPlumePixels(imageData.data, canvas, intensity, D, haloLength);
        
        // Apply the rendered pixels to the canvas
        ctx.putImageData(imageData, 0, 0);
        return canvas;
    }

    /**
     * Render individual pixels of the exhaust plume using physics-based calculations.
     *
     * @param {Uint8ClampedArray} data - Raw pixel data array
     * @param {HTMLCanvasElement} canvas - Target canvas
     * @param {number} intensity - Thruster intensity
     * @param {number} D - Nozzle diameter
     * @param {number} haloLength - Maximum plume length
     */
    renderPlumePixels(data, canvas, intensity, D, haloLength) {
        // Base color: exhaust gases appear yellowish-white
        const baseColor = [255, 255, 220];
        const centerY = canvas.height / 2;

        // Iterate through every pixel in the plume texture
        for (let py = 0; py < canvas.height; py++) {
            for (let px = 0; px < canvas.width; px++) {
                const x = px;                          // Axial distance from nozzle
                const r = Math.abs(py - centerY);     // Radial distance from centerline

                // Skip pixels outside the plume boundaries
                // Plume expands at 30° half-angle (60° total cone)
                if (x > haloLength || r > x * RocketSim.RCS.CONE_ANGLE_30) {
                    continue;
                }

                // Calculate pixel properties using gas dynamics model
                const { opacity, color } = this.calculatePlumePixel(x, r, D, intensity, baseColor);
                
                // Apply calculated values to pixel data
                const index = (py * canvas.width + px) * 4;  // RGBA index
                data[index] = Math.min(255, color[0]);        // Red
                data[index + 1] = Math.min(255, color[1]);    // Green
                data[index + 2] = Math.min(255, color[2]);    // Blue
                data[index + 3] = Math.min(255, opacity * 255); // Alpha
            }
        }
    }

    /**
     * Calculate the optical properties of a single plume pixel.
     *
     * @param {number} x - Axial distance from nozzle exit
     * @param {number} r - Radial distance from plume centerline
     * @param {number} D - Nozzle diameter (reference scale)
     * @param {number} intensity - Thruster intensity (0.0 to 1.0)
     * @param {Array} baseColor - RGB base color for hot exhaust
     * @returns {Object} Calculated opacity and color values
     */
    calculatePlumePixel(x, r, D, intensity, baseColor) {
        let finalOpacity = 0;
        let finalColor = [0, 0, 0];

        // Axial intensity falloff: exhaust density decreases with distance
        // Uses inverse square law approximation for gas expansion
        const xD = Math.max(1, x / D);
        const axialFalloff = 1.0 / (xD * xD);

        // === CORE JET CONTRIBUTION ===
        // High-velocity, high-temperature center region
        // 15° half-angle represents the potential core where velocity is nearly constant
        if (r <= x * RocketSim.RCS.CONE_ANGLE_15 && x <= 25 * D) {
            // Gaussian radial profile for smooth intensity distribution
            const sigma = 0.3 * x * RocketSim.RCS.CONE_ANGLE_15;
            const radialFalloff = sigma > 0 ? Math.exp(-(r * r) / (sigma * sigma)) : (r === 0 ? 1 : 0);
            const coreOpacity = intensity * 0.8 * radialFalloff;

            finalOpacity += coreOpacity;
            for (let i = 0; i < 3; i++) {
                finalColor[i] += baseColor[i] * axialFalloff * coreOpacity;
            }
        }

        // === HALO CONTRIBUTION ===
        // 30° expansion angle for fully developed supersonic jet
        const sigma = 0.6 * x * RocketSim.RCS.CONE_ANGLE_30;
        const radialFalloff = sigma > 0 ? Math.exp(-(r * r) / (sigma * sigma)) : (r === 0 ? 1 : 0);
        const haloOpacity = intensity * 0.15 * radialFalloff;

        finalOpacity += haloOpacity;
        for (let i = 0; i < 3; i++) {
            // Halo is dimmer and slightly cooler than core
            finalColor[i] += baseColor[i] * axialFalloff * haloOpacity * 0.5;
        }

        return { opacity: finalOpacity, color: finalColor };
    }

    // ====================================================================
    // GRAPHICS RENDERING SYSTEM
    // ====================================================================
    
    /**
     * Main rendering function called every frame.
     * Draws all visual elements: background, starfield, rocket, and effects.
     */
    render() {
        // Create motion blur trail effect by partially clearing the canvas
        // Higher values = faster fade, cleaner trails
        this.ctx.fillStyle = `rgba(0, 0, 0, ${RocketSim.VISUAL.TRAIL_FADE})`;
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Draw background elements first
        this.drawStarfield();
        
        // Draw main rocket (this handles all rocket-related rendering)
        this.drawRocket();
    }

    /**
     * Draw the complete rocket assembly with all components.
     * Uses save/restore to isolate coordinate transformations.
     */
    drawRocket() {
        this.ctx.save();
        
        // Transform coordinate system to rocket's position and orientation
        this.ctx.translate(this.rocket.x, this.rocket.y);
        this.ctx.rotate(this.rocket.angle);
        
        // Draw all rocket components in local coordinate system
        this.drawTargetIndicator();   // Blue guidance arrow
        this.drawRCSPlumes();         // Exhaust plumes
        this.drawRocketBody();        // Main vehicle structure
        this.drawRCSPods();           // Thruster pods
        
        this.ctx.restore();
    }

    /**
     * Draw target orientation indicator.
     * Shows where the automatic guidance system is trying to point the rocket.
     */
    drawTargetIndicator() {
        this.ctx.save();
        
        // Rotate to show target direction relative to current orientation
        this.ctx.rotate(this.guidance.targetAngle - this.rocket.angle);
        
        // Draw subtle blue arrow indicating target direction
        this.ctx.strokeStyle = "rgba(96, 165, 250, 0.5)";
        this.ctx.lineWidth = 2;
        this.ctx.beginPath();
        this.ctx.moveTo(0, -RocketSim.ROCKET.LENGTH * 0.6);
        this.ctx.lineTo(0, -RocketSim.ROCKET.LENGTH * 0.8);
        this.ctx.stroke();
        
        this.ctx.restore();
    }

    /**
     * Render RCS thruster exhaust plumes.
     * Uses pre-generated textures and additive blending for glow effects.
     */
    drawRCSPlumes() {
        // Only draw plumes if thrusters are actually firing
        if (!this.rcs.activePulse || this.rcs.activePulse.intensity <= 0) {
            return;
        }

        const intensity = this.rcs.activePulse.intensity;
        const direction = this.rcs.activePulse.direction;
        
        // Select appropriate pre-rendered texture based on intensity
        const cacheIndex = Math.round(intensity * (this.plumeCache.length - 1));
        const plumeImage = this.plumeCache[cacheIndex];
        
        if (!plumeImage) return;

        // Thruster locations on the rocket body
        const side = RocketSim.ROCKET.WIDTH / 2;

        // Use additive blending for realistic light emission
        this.ctx.globalCompositeOperation = 'lighter';
        
        // Draw plumes based on rotation direction
        // Positive direction: clockwise rotation (left thrusters fire down, right up)
        if (direction > 0) {
            this.drawPlumeAt(plumeImage, -side, -65, -1);  // Left thruster fires backward
            this.drawPlumeAt(plumeImage, side, 65, 1);     // Right thruster fires forward
        } else {
            // Negative direction: counterclockwise rotation
            this.drawPlumeAt(plumeImage, side, -65, 1);    // Right thruster fires backward
            this.drawPlumeAt(plumeImage, -side, 65, -1);   // Left thruster fires forward
        }

        // Restore normal blending mode
        this.ctx.globalCompositeOperation = 'source-over';
    }

    /**
     * Draw a single exhaust plume at specified location and orientation.
     * 
     * @param {HTMLCanvasElement} plumeImage - Pre-rendered plume texture
     * @param {number} x - X position relative to rocket center
     * @param {number} y - Y position relative to rocket center
     * @param {number} direction - Plume direction (-1 = backward, 1 = forward)
     */
    drawPlumeAt(plumeImage, x, y, direction) {
        this.ctx.save();
        
        // Position the plume at the thruster location
        this.ctx.translate(x, y);
        
        // Flip plume direction if needed
        if (direction < 0) {
            this.ctx.scale(-1, 1);
        }
        
        // Draw centered on the thruster nozzle
        this.ctx.drawImage(plumeImage, 0, -plumeImage.height / 2);
        
        this.ctx.restore();
    }

    /**
     * Draw the main rocket body with realistic shading and details.
     * Includes fuselage, nose cone, and propulsion system.
     */
    drawRocketBody() {
        const w = RocketSim.ROCKET.WIDTH;
        const h = RocketSim.ROCKET.LENGTH;
        
        // Add shadow for depth perception
        this.ctx.shadowColor = 'rgba(0, 0, 0, 0.5)';
        this.ctx.shadowBlur = 20;
        this.ctx.shadowOffsetX = 5;
        this.ctx.shadowOffsetY = 5;
        
        // Main body with metallic gradient effect
        const bodyGradient = this.ctx.createLinearGradient(-w/2, 0, w/2, 0);
        bodyGradient.addColorStop(0, '#1e40af');    // Darker blue on sides
        bodyGradient.addColorStop(0.5, '#2563eb');  // Brighter blue in center
        bodyGradient.addColorStop(1, '#1e40af');    // Darker blue on sides
        
        this.ctx.fillStyle = bodyGradient;
        this.ctx.fillRect(-w/2, -h/2, w, h);
        
        // Bottom end cap with curved profile
        const capHeight = w * 0.4;
        this.ctx.beginPath();
        this.ctx.moveTo(-w/2, h/2);
        this.ctx.quadraticCurveTo(0, h/2 + capHeight, w/2, h/2);
        this.ctx.lineTo(-w/2, h/2);
        this.ctx.closePath();
        this.ctx.fill();
        
        // Nose cone
        const noseGradient = this.ctx.createRadialGradient(0, -h/2 - 20, 0, 0, -h/2 - 20, 30);
        noseGradient.addColorStop(0, '#60a5fa');    // Bright center
        noseGradient.addColorStop(1, '#2563eb');    // Darker edges
        this.ctx.fillStyle = noseGradient;
        
        this.ctx.beginPath();
        this.ctx.moveTo(-w/2, -h/2);
        this.ctx.quadraticCurveTo(-w/2, -h/2 - 35, 0, -h/2 - RocketSim.ROCKET.NOSE_CONE_LENGTH);
        this.ctx.quadraticCurveTo(w/2, -h/2 - 35, w/2, -h/2);
        this.ctx.fill();
        
        // Add surface details and propulsion system
        this.drawRocketDetails();
        this.drawRocketNozzle();
    }

    /**
     * Add surface details to the rocket.
     */
    drawRocketDetails() {
        const w = RocketSim.ROCKET.WIDTH;
        const h = RocketSim.ROCKET.LENGTH;
        
        // Clear shadows for detail work
        this.ctx.strokeStyle = '#1e293b';
        this.ctx.lineWidth = 1;
        this.ctx.shadowColor = 'transparent';
        this.ctx.fillStyle = '#475569';
        
        // Draw panel lines and structural details
        for (let i = -3; i <= 3; i++) {
            const y = i * h / 8;
            
            // Panel lines
            this.ctx.beginPath();
            this.ctx.moveTo(-w/2, y);
            this.ctx.lineTo(w/2, y);
            this.ctx.stroke();
            
            // Panel intersections
            for (let j = -1; j <= 1; j += 2) {
                const x = j * w / 3.5;
                this.ctx.beginPath();
                this.ctx.arc(x, y, 1.5, 0, Math.PI * 2);
                this.ctx.fill();
            }
        }
    }

    /**
     * Draw the main propulsion system.
     */
    drawRocketNozzle() {
        const w = RocketSim.ROCKET.WIDTH;
        const h = RocketSim.ROCKET.LENGTH;
        const capHeight = w * 0.4;
        
        // Nozzle geometry based on typical rocket engine proportions
        const chamberHeight = 12;          // Combustion chamber height
        const chamberRadius = w * 0.16;    // Chamber radius
        const throatRadius = w * 0.10;     // Throat radius
        const exitRadius = w * 0.35;       // Exit radius
        const bellHeight = h * 0.18;       // Expansion bell height
        
        // Vertical positions
        const y0 = h/2 + capHeight * 0.6;                // Chamber top
        const yThroat = y0 + chamberHeight;              // Throat location
        const yExit = yThroat + bellHeight;              // Nozzle exit
        
        // Create gradients
        const bellGradient = this.ctx.createLinearGradient(-exitRadius, 0, exitRadius, 0);
        bellGradient.addColorStop(0, '#1a202c');
        bellGradient.addColorStop(0.4, '#4a5568');
        bellGradient.addColorStop(0.6, '#2d3748');
        bellGradient.addColorStop(1, '#1a202c');
        
        const chamberGradient = this.ctx.createLinearGradient(-chamberRadius, 0, chamberRadius, 0);
        chamberGradient.addColorStop(0, '#1a202c');
        chamberGradient.addColorStop(0.3, '#4a5568');
        chamberGradient.addColorStop(0.7, '#2d3748');
        chamberGradient.addColorStop(1, '#1a202c');
        
        // === COMBUSTION CHAMBER ===
        this.ctx.fillStyle = chamberGradient;
        
        // Chamber top end cap
        this.ctx.beginPath();
        this.ctx.ellipse(0, y0, chamberRadius, 3, 0, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // Chamber cylindrical section
        this.ctx.fillRect(-chamberRadius, y0, chamberRadius * 2, chamberHeight);
        
        // Chamber bottom transition
        this.ctx.beginPath();
        this.ctx.ellipse(0, y0 + chamberHeight, chamberRadius, 3, 0, 0, 2 * Math.PI);
        this.ctx.fill();
        
        // Add chamber detail lines
        this.ctx.strokeStyle = '#1e293b';
        this.ctx.lineWidth = 1;
        for (let i = 0; i < 2; i++) {
            const y = y0 + 3 + i * 6;
            this.ctx.beginPath();
            this.ctx.moveTo(-chamberRadius, y);
            this.ctx.lineTo(chamberRadius, y);
            this.ctx.stroke();
        }
        
        // === NOZZLE EXPANSION BELL ===
        this.ctx.fillStyle = bellGradient;
        this.ctx.beginPath();
        
        // Left side of bell
        this.ctx.moveTo(-throatRadius, yThroat);
        this.ctx.bezierCurveTo(
            -throatRadius - w * 0.08, yThroat + bellHeight * 0.15,
            -exitRadius + w * 0.05, yThroat + bellHeight * 0.75,
            -exitRadius, yExit
        );
        
        // Bottom edge
        this.ctx.lineTo(exitRadius, yExit);
        
        // Right side of bell (mirror of left)
        this.ctx.bezierCurveTo(
            exitRadius - w * 0.05, yThroat + bellHeight * 0.75,
            throatRadius + w * 0.08, yThroat + bellHeight * 0.15,
            throatRadius, yThroat
        );
        
        this.ctx.closePath();
        this.ctx.fill();
        
        // === NOZZLE EXIT ===
        // Draw the dark interior of the nozzle exit
        const openingGradient = this.ctx.createRadialGradient(0, yExit - 2, 1, 0, yExit, exitRadius);
        openingGradient.addColorStop(0, '#273040');
        openingGradient.addColorStop(0.7, '#151b26');
        openingGradient.addColorStop(1, '#0b111f');
        
        this.ctx.fillStyle = openingGradient;
        this.ctx.beginPath();
        this.ctx.ellipse(0, yExit, exitRadius, 2, 0, 0, 2 * Math.PI);
        this.ctx.fill();
    }

    /**
     * Draw small RCS thruster pods on the sides of the rocket.
     */
    drawRCSPods() {
        const side = RocketSim.ROCKET.WIDTH / 2;
        const podSize = 4;
        
        this.ctx.fillStyle = '#374151';
        
        // Four thruster pods: two on each side at different heights
        this.ctx.fillRect(side - podSize, -65 - podSize/2, podSize, podSize);  // Right forward
        this.ctx.fillRect(side - podSize, 65 - podSize/2, podSize, podSize);   // Right aft
        this.ctx.fillRect(-side, -65 - podSize/2, podSize, podSize);           // Left forward
        this.ctx.fillRect(-side, 65 - podSize/2, podSize, podSize);            // Left aft
    }

    /**
     * Draw animated starfield background.
     * Creates the illusion of movement through space.
     */
    drawStarfield() {
        this.ctx.fillStyle = "#fff";
        
        for (const star of this.starfield) {
            // Move stars downward to simulate forward motion
            star.y += star.speed;
            
            // Wrap stars around when they go off-screen
            if (star.y > this.canvas.height) {
                star.y = 0;
                star.x = Math.random() * this.canvas.width;
            }
            
            // Draw individual star
            this.ctx.beginPath();
            this.ctx.arc(star.x, star.y, star.radius, 0, 2 * Math.PI);
            this.ctx.fill();
        }
    }

    // ====================================================================
    // USER INPUT HANDLING
    // ====================================================================
    
    /**
     * Set up mouse and touch event handlers for rocket interaction.
     * Supports both desktop and mobile devices.
     */
    setupInput() {
        // Desktop mouse events
        this.canvas.addEventListener('mousedown', e => this.handlePointerStart(e));
        this.canvas.addEventListener('mousemove', e => this.handlePointerMove(e));
        this.canvas.addEventListener('mouseup', e => this.handlePointerEnd(e));
        
        // Mobile touch events
        this.canvas.addEventListener('touchstart', e => {
            e.preventDefault();  // Prevent scrolling and zoom
            this.handlePointerStart(e.touches[0]);
        });
        this.canvas.addEventListener('touchmove', e => {
            e.preventDefault();
            this.handlePointerMove(e.touches[0]);
        });
        this.canvas.addEventListener('touchend', e => {
            e.preventDefault();
            this.handlePointerEnd(e.changedTouches[0]);
        });
    }

    /**
     * Handle start of pointer interaction (mouse down or touch start).
     * Determines whether to start dragging the rocket.
     * 
     * @param {MouseEvent|Touch} pointer - Pointer event object
     */
    handlePointerStart(pointer) {
        const pos = this.getPointerPosition(pointer);
        
        // Check if the user clicked/touched the rocket
        if (this.isPointerOverRocket(pos)) {
            this.startDragging(pos);
        }
    }

    /**
     * Handle pointer movement (mouse move or touch move).
     * Updates rocket orientation during drag or cursor appearance.
     * 
     * @param {MouseEvent|Touch} pointer - Pointer event object
     */
    handlePointerMove(pointer) {
        const pos = this.getPointerPosition(pointer);
        
        // Update rocket angle if currently dragging
        if (this.rocket.isDragging) {
            this.updateDragAngle(pos);
        }
        
        // Update cursor to indicate interactive elements
        this.updateCursorStyle(pos);
    }

    /**
     * Handle end of pointer interaction (mouse up or touch end).
     * Either stops dragging or sets a new target direction.
     * 
     * @param {MouseEvent|Touch} pointer - Pointer event object
     */
    handlePointerEnd(pointer) {
        if (this.rocket.isDragging) {
            // Stop manual control and let automatic guidance take over
            this.rocket.isDragging = false;
        } else {
            // Set new target direction for automatic guidance
            const pos = this.getPointerPosition(pointer);
            this.setTargetAngle(pos);
        }
    }

    /**
     * Convert pointer event coordinates to canvas coordinates.
     * Accounts for canvas position and scaling.
     * 
     * @param {MouseEvent|Touch} pointer - Pointer event object
     * @returns {Object} Canvas coordinates {x, y}
     */
    getPointerPosition(pointer) {
        const rect = this.canvas.getBoundingClientRect();
        return {
            x: pointer.clientX - rect.left,
            y: pointer.clientY - rect.top
        };
    }

    /**
     * Check if pointer is over the rocket (for interaction detection).
     * Uses elliptical hit detection with expanded boundaries for easier selection.
     * 
     * @param {Object} pos - Pointer position {x, y}
     * @returns {boolean} True if pointer is over rocket
     */
    isPointerOverRocket(pos) {
        // Calculate offset from rocket center
        const dx = pos.x - this.rocket.x;
        const dy = pos.y - this.rocket.y;
        
        // Transform to rocket's local coordinate system
        const cos = Math.cos(-this.rocket.angle);
        const sin = Math.sin(-this.rocket.angle);
        const localX = dx * cos - dy * sin;
        const localY = dx * sin + dy * cos;
        
        // Define hitbox dimensions (slightly larger than visual for easier clicking)
        const width = RocketSim.ROCKET.WIDTH / 2 * RocketSim.CONTROL.HITBOX_PADDING_X;
        const height = (RocketSim.ROCKET.LENGTH / 2 + RocketSim.ROCKET.NOSE_CONE_LENGTH) * RocketSim.CONTROL.HITBOX_PADDING_Y;
        
        // Elliptical hit detection
        return (localX * localX) / (width * width) + (localY * localY) / (height * height) < 1;
    }

    /**
     * Start dragging operation.
     * Records the angular offset between rocket and mouse for natural dragging feel.
     * 
     * @param {Object} pos - Pointer position {x, y}
     */
    startDragging(pos) {
        this.rocket.isDragging = true;
        
        // Calculate angle from rocket center to mouse position
        // Add π/2 because rocket "up" is negative Y direction
        const mouseAngle = Math.atan2(pos.y - this.rocket.y, pos.x - this.rocket.x) + Math.PI / 2;
        
        // Store the angular offset for smooth dragging
        this.rocket.dragAngleOffset = this.normalizeAngle(this.rocket.angle - mouseAngle);
    }

    /**
     * Update rocket angle during drag operation.
     * Maintains the initial angular offset for natural feel.
     * 
     * @param {Object} pos - Pointer position {x, y}
     */
    updateDragAngle(pos) {
        // Calculate current mouse angle
        const mouseAngle = Math.atan2(pos.y - this.rocket.y, pos.x - this.rocket.x) + Math.PI / 2;
        
        // Apply the stored offset to maintain smooth dragging
        this.rocket.angle = this.normalizeAngle(mouseAngle + this.rocket.dragAngleOffset);
    }

    /**
     * Set target angle for automatic guidance system.
     * Converts mouse position to desired rocket orientation.
     * 
     * @param {Object} pos - Pointer position {x, y}
     */
    setTargetAngle(pos) {
        // Calculate angle from rocket to target position
        this.guidance.targetAngle = this.normalizeAngle(
            Math.atan2(pos.y - this.rocket.y, pos.x - this.rocket.x) + Math.PI / 2
        );
    }

    /**
     * Update cursor style based on pointer position.
     * Provides visual feedback for interactive elements.
     * 
     * @param {Object} pos - Pointer position {x, y}
     */
    updateCursorStyle(pos) {
        if (this.isPointerOverRocket(pos)) {
            // Show grab cursor when over rocket
            this.canvas.style.cursor = this.rocket.isDragging ? 'grabbing' : 'grab';
        } else {
            // Default cursor elsewhere
            this.canvas.style.cursor = 'default';
        }
    }

    // ====================================================================
    // CANVAS SETUP AND SIMULATION LOOP
    // ====================================================================
    
    /**
     * Configure canvas for full-screen rendering and handle window resizing.
     */
    setupCanvas() {
        const resize = () => {
            // Set canvas to full window size
            this.canvas.width = window.innerWidth;
            this.canvas.height = window.innerHeight;
            
            // Center rocket in viewport
            this.rocket.x = this.canvas.width / 2;
            this.rocket.y = this.canvas.height / 2;
            
            // Regenerate starfield for new dimensions
            this.initializeStarfield();
        };
        
        // Initial setup and window resize handler
        resize();
        window.addEventListener('resize', resize);
    }

    /**
     * Generate random starfield for space background.
     * Star count scales with screen size for consistent density.
     */
    initializeStarfield() {
        this.starfield = [];
        const starCount = Math.round(this.canvas.width * this.canvas.height * RocketSim.VISUAL.STAR_DENSITY);
        
        for (let i = 0; i < starCount; i++) {
            this.starfield.push({
                x: Math.random() * this.canvas.width,     // Random X position
                y: Math.random() * this.canvas.height,    // Random Y position
                speed: 1 + Math.random() * 3,             // Variable speed for depth
                radius: 0.5 + Math.random() * 1.5         // Variable size for realism
            });
        }
    }

    /**
     * Start the main simulation loop.
     * Initializes timing and begins continuous updates.
     */
    startSimulation() {
        this.physics.lastTime = performance.now();
        this.simulationLoop();
    }

    /**
     * Main simulation loop - runs every frame.
     * Handles timing, physics updates, and rendering.
     */
    simulationLoop() {
        const now = performance.now();
        const rawDeltaTime = (now - this.physics.lastTime) / 1000;
        
        // Clamp timestep to prevent numerical instability during lag
        const deltaTime = Math.min(rawDeltaTime, RocketSim.PHYSICS.MAX_FRAME_TIME);
        this.physics.lastTime = now;

        // Update simulation state
        this.updatePhysics(deltaTime);
        
        // Render current frame
        this.render();
        
        // Schedule next frame
        requestAnimationFrame(() => this.simulationLoop());
    }
}

// ====================================================================
// APPLICATION STARTUP
// ====================================================================

/**
 * Initialize the simulation when the page loads.
 * Waits for DOM to be ready before starting.
 */
window.addEventListener('DOMContentLoaded', () => {
    // Create and start the rocket simulation
    new RocketSim();
});