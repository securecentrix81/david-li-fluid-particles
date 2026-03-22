//'use strict';
 
var Simulator = (function () {

    function swap(object, a, b) {
        var temp = object[a];
        object[a] = object[b];
        object[b] = temp;
    }

    function vec3Length(v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    function getNow() {
        if (typeof performance !== 'undefined' && performance.now) {
            return performance.now();
        }
        return Date.now();
    }

    function getScreenOrientationAngle() {
        if (typeof window === 'undefined') return 0;

        if (window.screen && window.screen.orientation && typeof window.screen.orientation.angle === 'number') {
            return window.screen.orientation.angle;
        }

        if (typeof window.orientation === 'number') {
            return window.orientation;
        }

        return 0;
    }

    function rotateDeviceXYToScreenXY(x, y, angleDegrees) {
        var angle = ((Math.round(angleDegrees / 90) * 90) % 360 + 360) % 360;

        if (angle === 90) {
            return [-y, x];
        } else if (angle === 180) {
            return [-x, -y];
        } else if (angle === 270) {
            return [y, -x];
        }

        return [x, y];
    }

    function identityMatrix4() {
        return new Float32Array([
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
        ]);
    }

    // simulation grid dimensions and resolution
    // all particles are in world position space ([0, 0, 0], [GRID_WIDTH, GRID_HEIGHT, GRID_DEPTH])
    //
    // when doing most grid operations, we transform positions from world position space into the grid position space
    // ([0, 0, 0], [GRID_RESOLUTION_X, GRID_RESOLUTION_Y, GRID_RESOLUTION_Z])
    //
    // in grid space, cell boundaries are simply at integer values
    //
    // we emulate 3D textures with tiled 2D textures
    // so the z slices of a 3D texture are laid out along the x axis
    // the 2D dimensions of a 3D texture are therefore [width * depth, height]
    //
    // we use a staggered MAC grid
    // this means the velocity grid width = grid width + 1 and velocity grid height = grid height + 1 and velocity grid depth = grid depth + 1
    // a scalar for cell [i, j, k] is positionally located at [i + 0.5, j + 0.5, k + 0.5]
    // x velocity for cell [i, j, k] is positionally located at [i, j + 0.5, k + 0.5]
    // y velocity for cell [i, j, k] is positionally located at [i + 0.5, j, k + 0.5]
    // z velocity for cell [i, j, k] is positionally located at [i + 0.5, j + 0.5, k]
    //
    // the boundaries are the boundaries of the grid
    // a grid cell can either be fluid, air (these are tracked by markerTexture) or is a wall (implicit by position)

    function Simulator(wgl, onLoaded) {
        this.wgl = wgl;

        this.particlesWidth = 0;
        this.particlesHeight = 0;

        this.gridWidth = 0;
        this.gridHeight = 0;
        this.gridDepth = 0;

        this.gridResolutionX = 0;
        this.gridResolutionY = 0;
        this.gridResolutionZ = 0;

        this.particleDensity = 0;

        this.velocityTextureWidth = 0;
        this.velocityTextureHeight = 0;

        this.scalarTextureWidth = 0;
        this.scalarTextureHeight = 0;

        this.halfFloatExt = this.wgl.getExtension('OES_texture_half_float');
        this.wgl.getExtension('OES_texture_half_float_linear');

        this.simulationNumberType = this.halfFloatExt ? this.halfFloatExt.HALF_FLOAT_OES : this.wgl.FLOAT;

        ///////////////////////////////////////////////////////
        // simulation parameters

        this.flipness = 0.99; // 0 is full PIC, 1 is full FLIP
        this.frameNumber = 0; // used for motion randomness

        // gravity configuration
        // keep the same magnitude as the old hardcoded shader gravity
        this.defaultGravity = [0.0, -40.0, 0.0];
        this.currentGravity = this.defaultGravity.slice(0);
        this.gravitySmoothing = 0.15;

        // device motion / accelerometer state
        this.useAccelerometerGravity = true;
        this.filteredDeviceGravity = null;
        this.deviceGravityTimestamp = 0;
        this.deviceGravityMaxAge = 500; // ms
        this.deviceMotionFilterStrength = 5.85;

        this.viewMatrix = identityMatrix4();
        this.hasViewMatrix = false;

        this.deviceMotionSupported = (typeof window !== 'undefined' && 'DeviceMotionEvent' in window);
        this.deviceMotionPermissionState = this.deviceMotionSupported ? 'unknown' : 'unsupported';
        this.deviceMotionListening = false;

        this._boundHandleDeviceMotion = this._handleDeviceMotion.bind(this);
        this._boundBootstrapDeviceMotion = null;

        ///////////////////////////////////////////////////////
        // simulation objects

        this.quadVertexBuffer = wgl.createBuffer();
        wgl.bufferData(
            this.quadVertexBuffer,
            wgl.ARRAY_BUFFER,
            new Float32Array([-1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, 1.0]),
            wgl.STATIC_DRAW
        );

        this.simulationFramebuffer = wgl.createFramebuffer();
        this.particleVertexBuffer = wgl.createBuffer();

        this.particlePositionTexture = wgl.createTexture();
        this.particlePositionTextureTemp = wgl.createTexture();

        this.particleVelocityTexture = wgl.createTexture();
        this.particleVelocityTextureTemp = wgl.createTexture();

        this.particleRandomTexture = wgl.createTexture(); // contains a random normalized direction for each particle

        ////////////////////////////////////////////////////
        // create simulation textures

        this.velocityTexture = wgl.createTexture();
        this.tempVelocityTexture = wgl.createTexture();
        this.originalVelocityTexture = wgl.createTexture();
        this.weightTexture = wgl.createTexture();

        this.markerTexture = wgl.createTexture(); // marks fluid/air, 1 if fluid, 0 if air
        this.divergenceTexture = wgl.createTexture();
        this.pressureTexture = wgl.createTexture();
        this.tempSimulationTexture = wgl.createTexture();

        ////////////////////////////////////////////////////
        // best-effort accelerometer startup
        // On iOS this usually requires a user gesture, so we install a one-time bootstrap.
        if (typeof window !== 'undefined' && this.deviceMotionSupported) {
            if (window.DeviceMotionEvent && typeof window.DeviceMotionEvent.requestPermission === 'function') {
                this._installDeviceMotionBootstrap();
            } else {
                this.deviceMotionPermissionState = 'granted';
                this._attachDeviceMotionListener();
            }
        }

        /////////////////////////////
        // load programs

        wgl.createProgramsFromFiles({
            transferToGridProgram: {
                vertexShader: 'shaders/transfertogrid.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/transfertogrid.frag'],
                attributeLocations: { 'a_textureCoordinates': 0 }
            },
            normalizeGridProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: 'shaders/normalizegrid.frag',
                attributeLocations: { 'a_position': 0 }
            },
            markProgram: {
                vertexShader: 'shaders/mark.vert',
                fragmentShader: 'shaders/mark.frag',
                attributeLocations: { 'a_textureCoordinates': 0 }
            },
            addForceProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/addforce.frag'],
                attributeLocations: { 'a_position': 0 }
            },
            enforceBoundariesProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/enforceboundaries.frag'],
                attributeLocations: { 'a_textureCoordinates': 0 }
            },
            extendVelocityProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: 'shaders/extendvelocity.frag',
                attributeLocations: { 'a_textureCoordinates': 0 }
            },
            transferToParticlesProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/transfertoparticles.frag'],
                attributeLocations: { 'a_position': 0 }
            },
            divergenceProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/divergence.frag'],
                attributeLocations: { 'a_position': 0 }
            },
            jacobiProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/jacobi.frag'],
                attributeLocations: { 'a_position': 0 }
            },
            subtractProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/subtract.frag'],
                attributeLocations: { 'a_position': 0 }
            },
            advectProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: ['shaders/common.frag', 'shaders/advect.frag'],
                attributeLocations: { 'a_position': 0 }
            },
            copyProgram: {
                vertexShader: 'shaders/fullscreen.vert',
                fragmentShader: 'shaders/copy.frag',
                attributeLocations: { 'a_position': 0 }
            }
        }, (function (programs) {
            for (var programName in programs) {
                this[programName] = programs[programName];
            }

            onLoaded();
        }).bind(this));
    }

    Simulator.prototype._installDeviceMotionBootstrap = function () {
        if (typeof window === 'undefined') return;
        if (!this.deviceMotionSupported) return;
        if (this._boundBootstrapDeviceMotion) return;

        var self = this;

        this._boundBootstrapDeviceMotion = function () {
            self._removeDeviceMotionBootstrap();
            self.enableDeviceGravity();
        };

        window.addEventListener('pointerup', this._boundBootstrapDeviceMotion, false);
        window.addEventListener('touchend', this._boundBootstrapDeviceMotion, false);
        window.addEventListener('click', this._boundBootstrapDeviceMotion, false);
    };

    Simulator.prototype._removeDeviceMotionBootstrap = function () {
        if (typeof window === 'undefined') return;
        if (!this._boundBootstrapDeviceMotion) return;

        window.removeEventListener('pointerup', this._boundBootstrapDeviceMotion, false);
        window.removeEventListener('touchend', this._boundBootstrapDeviceMotion, false);
        window.removeEventListener('click', this._boundBootstrapDeviceMotion, false);

        this._boundBootstrapDeviceMotion = null;
    };

    Simulator.prototype._attachDeviceMotionListener = function () {
        if (typeof window === 'undefined') return;
        if (this.deviceMotionListening) return;

        window.addEventListener('devicemotion', this._boundHandleDeviceMotion, false);
        this.deviceMotionListening = true;
    };

    Simulator.prototype.enableDeviceGravity = function () {
        var self = this;

        this.useAccelerometerGravity = true;

        if (!this.deviceMotionSupported || typeof window === 'undefined') {
            this.deviceMotionPermissionState = 'unsupported';
            return Promise.resolve(false);
        }

        if (this.deviceMotionListening) {
            this.deviceMotionPermissionState = 'granted';
            return Promise.resolve(true);
        }

        if (window.DeviceMotionEvent && typeof window.DeviceMotionEvent.requestPermission === 'function') {
            return window.DeviceMotionEvent.requestPermission().then(function (state) {
                if (state === 'granted') {
                    self.deviceMotionPermissionState = 'granted';
                    self._attachDeviceMotionListener();
                    return true;
                }

                self.deviceMotionPermissionState = 'denied';
                return false;
            }).catch(function () {
                self.deviceMotionPermissionState = 'denied';
                return false;
            });
        }

        this.deviceMotionPermissionState = 'granted';
        this._attachDeviceMotionListener();
        return Promise.resolve(true);
    };

    Simulator.prototype.disableDeviceGravity = function () {
        this.useAccelerometerGravity = false;
    };

    Simulator.prototype.setDefaultGravity = function (gravityVector) {
        this.defaultGravity = [
            gravityVector[0],
            gravityVector[1],
            gravityVector[2]
        ];

        if (!this.useAccelerometerGravity) {
            this.currentGravity = this.defaultGravity.slice(0);
        }
    };

    Simulator.prototype.setViewMatrix = function (viewMatrix) {
        if (!viewMatrix || viewMatrix.length < 16) return;

        for (var i = 0; i < 16; ++i) {
            this.viewMatrix[i] = viewMatrix[i];
        }

        this.hasViewMatrix = true;
    };

        Simulator.prototype._handleDeviceMotion = function (event) {
    // We need both acceleration types to separate gravity from movement
    if (!event || !event.accelerationIncludingGravity || !event.acceleration) return;

    // 1. Acceleration WITHOUT Gravity (accelnograv)
    var ax = Number(event.acceleration.x || 0);
    var ay = Number(event.acceleration.y || 0);
    var az = Number(event.acceleration.z || 0);

    // 2. Acceleration WITH Gravity
    var gx = Number(event.accelerationIncludingGravity.x || 0);
    var gy = Number(event.accelerationIncludingGravity.y || 0);
    var gz = Number(event.accelerationIncludingGravity.z || 0);

    if (!isFinite(ax) || !isFinite(gx)) return;

    var mul = this.deviceMotionFilterStrength;

    // Formula: (accelnograv * mul) + gravnoaccel
    // Note: gravnoaccel is (IncludingGravity - LinearAcceleration)
    this.filteredDeviceGravity = [
        (ax * mul) + (gx - ax),
        (ay * mul) + (gy - ay),
        (az * mul) + (gz - az)
    ];

    this.deviceGravityTimestamp = getNow();
};

    Simulator.prototype._deviceGravityToCameraSpace = function (deviceGravity) {
        var rotatedXY = rotateDeviceXYToScreenXY(
            deviceGravity[0],
            deviceGravity[1],
            getScreenOrientationAngle()
        );

        return [
            rotatedXY[0],
            rotatedXY[1],
            deviceGravity[2]
        ];
    };

    // viewMatrix transforms world -> camera.
    // For a direction vector, camera -> world uses the transpose of the 3x3 rotation.
    Simulator.prototype._cameraVectorToWorldVector = function (cameraVector) {
        if (!this.hasViewMatrix) return null;

        var m = this.viewMatrix;
        var cx = cameraVector[0];
        var cy = cameraVector[1];
        var cz = cameraVector[2];

        return [
            m[0] * cx + m[1] * cy + m[2] * cz,
            m[4] * cx + m[5] * cy + m[6] * cz,
            m[8] * cx + m[9] * cy + m[10] * cz
        ];
    };

    Simulator.prototype._computeTargetGravity = function () {
    var defaultForce = this.defaultGravity.slice(0); // Usually [0, -40, 0]

    // Fallback if accelerometer is off or unavailable
    if (!this.useAccelerometerGravity || !this.deviceMotionListening || this.filteredDeviceGravity === null) {
        return defaultForce;
    }

    // Fallback if the sensor hasn't updated in a while (preventing "stuck" gravity)
    if ((getNow() - this.deviceGravityTimestamp) > this.deviceGravityMaxAge) {
        return defaultForce;
    }

    // 1. Transform the raw device data into Camera/Screen space
    var cameraGravity = this._deviceGravityToCameraSpace(this.filteredDeviceGravity);

    // 2. SCALE THE FORCE
    // Earth's gravity is ~9.81. Your simulator's gravity is ~40.0.
    // We create a multiplier so that holding the phone still results in 40.0 force units,
    // but shaking the phone (which might hit 20 or 30 m/s^2) results in 80.0 or 120.0 units.
    var earthGravityConstant = 9.80665;
    var simulatorScale = vec3Length(this.defaultGravity) / earthGravityConstant;

    cameraGravity[0] *= simulatorScale;
    cameraGravity[1] *= simulatorScale;
    cameraGravity[2] *= simulatorScale;

    // 3. Transform from Camera space to World space
    var worldGravity = this._cameraVectorToWorldVector(cameraGravity);

    return worldGravity || defaultForce;
};


    Simulator.prototype._updateCurrentGravity = function (timeStep) {
        var targetGravity = this._computeTargetGravity();

        var blend = 1.0 - Math.pow(1.0 - this.gravitySmoothing, Math.max(1.0, timeStep * 60.0));

        this.currentGravity[0] += (targetGravity[0] - this.currentGravity[0]) * blend;
        this.currentGravity[1] += (targetGravity[1] - this.currentGravity[1]) * blend;
        this.currentGravity[2] += (targetGravity[2] - this.currentGravity[2]) * blend;
    };

    // expects an array of [x, y, z] particle positions
    // gridSize and gridResolution are both [x, y, z]
    // particleDensity is particles per simulation grid cell
    Simulator.prototype.reset = function (particlesWidth, particlesHeight, particlePositions, gridSize, gridResolution, particleDensity) {
        var wgl = this.wgl;

        this.particlesWidth = particlesWidth;
        this.particlesHeight = particlesHeight;

        this.gridWidth = gridSize[0];
        this.gridHeight = gridSize[1];
        this.gridDepth = gridSize[2];

        this.gridResolutionX = gridResolution[0];
        this.gridResolutionY = gridResolution[1];
        this.gridResolutionZ = gridResolution[2];

        this.particleDensity = particleDensity;

        this.velocityTextureWidth = (this.gridResolutionX + 1) * (this.gridResolutionZ + 1);
        this.velocityTextureHeight = (this.gridResolutionY + 1);

        this.scalarTextureWidth = this.gridResolutionX * this.gridResolutionZ;
        this.scalarTextureHeight = this.gridResolutionY;

        ///////////////////////////////////////////////////////////
        // create particle data

        var particleTextureCoordinates = new Float32Array(this.particlesWidth * this.particlesHeight * 2);
        for (var y = 0; y < this.particlesHeight; ++y) {
            for (var x = 0; x < this.particlesWidth; ++x) {
                particleTextureCoordinates[(y * this.particlesWidth + x) * 2] = (x + 0.5) / this.particlesWidth;
                particleTextureCoordinates[(y * this.particlesWidth + x) * 2 + 1] = (y + 0.5) / this.particlesHeight;
            }
        }

        wgl.bufferData(this.particleVertexBuffer, wgl.ARRAY_BUFFER, particleTextureCoordinates, wgl.STATIC_DRAW);

        var particlePositionsData = new Float32Array(this.particlesWidth * this.particlesHeight * 4);
        var particleRandoms = new Float32Array(this.particlesWidth * this.particlesHeight * 4);

        for (var i = 0; i < this.particlesWidth * this.particlesHeight; ++i) {
            particlePositionsData[i * 4] = particlePositions[i][0];
            particlePositionsData[i * 4 + 1] = particlePositions[i][1];
            particlePositionsData[i * 4 + 2] = particlePositions[i][2];
            particlePositionsData[i * 4 + 3] = 0.0;

            var theta = Math.random() * 2.0 * Math.PI;
            var u = Math.random() * 2.0 - 1.0;

            particleRandoms[i * 4] = Math.sqrt(1.0 - u * u) * Math.cos(theta);
            particleRandoms[i * 4 + 1] = Math.sqrt(1.0 - u * u) * Math.sin(theta);
            particleRandoms[i * 4 + 2] = u;
            particleRandoms[i * 4 + 3] = 0.0;
        }

        wgl.rebuildTexture(this.particlePositionTexture, wgl.RGBA, wgl.FLOAT, this.particlesWidth, this.particlesHeight, particlePositionsData, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.NEAREST, wgl.NEAREST);
        wgl.rebuildTexture(this.particlePositionTextureTemp, wgl.RGBA, wgl.FLOAT, this.particlesWidth, this.particlesHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.NEAREST, wgl.NEAREST);

        wgl.rebuildTexture(this.particleVelocityTexture, wgl.RGBA, this.simulationNumberType, this.particlesWidth, this.particlesHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.NEAREST, wgl.NEAREST);
        wgl.rebuildTexture(this.particleVelocityTextureTemp, wgl.RGBA, this.simulationNumberType, this.particlesWidth, this.particlesHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.NEAREST, wgl.NEAREST);

        wgl.rebuildTexture(this.particleRandomTexture, wgl.RGBA, wgl.FLOAT, this.particlesWidth, this.particlesHeight, particleRandoms, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.NEAREST, wgl.NEAREST);

        ////////////////////////////////////////////////////
        // create simulation textures

        wgl.rebuildTexture(this.velocityTexture, wgl.RGBA, this.simulationNumberType, this.velocityTextureWidth, this.velocityTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);
        wgl.rebuildTexture(this.tempVelocityTexture, wgl.RGBA, this.simulationNumberType, this.velocityTextureWidth, this.velocityTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);
        wgl.rebuildTexture(this.originalVelocityTexture, wgl.RGBA, this.simulationNumberType, this.velocityTextureWidth, this.velocityTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);
        wgl.rebuildTexture(this.weightTexture, wgl.RGBA, this.simulationNumberType, this.velocityTextureWidth, this.velocityTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);

        wgl.rebuildTexture(this.markerTexture, wgl.RGBA, wgl.UNSIGNED_BYTE, this.scalarTextureWidth, this.scalarTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);
        wgl.rebuildTexture(this.divergenceTexture, wgl.RGBA, this.simulationNumberType, this.scalarTextureWidth, this.scalarTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);
        wgl.rebuildTexture(this.pressureTexture, wgl.RGBA, this.simulationNumberType, this.scalarTextureWidth, this.scalarTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);
        wgl.rebuildTexture(this.tempSimulationTexture, wgl.RGBA, this.simulationNumberType, this.scalarTextureWidth, this.scalarTextureHeight, null, wgl.CLAMP_TO_EDGE, wgl.CLAMP_TO_EDGE, wgl.LINEAR, wgl.LINEAR);
    };

    // You need to call reset() with correct parameters before simulating.
    // mouseVelocity, mouseRayOrigin, mouseRayDirection are arrays of 3 values.
    // Optional 5th argument: viewMatrix. If provided, it is used immediately for gravity alignment.
    Simulator.prototype.simulate = function (timeStep, mouseVelocity, mouseRayOrigin, mouseRayDirection, viewMatrix) {
        var wgl = this.wgl;

        mouseVelocity = mouseVelocity || [0.0, 0.0, 0.0];
        mouseRayOrigin = mouseRayOrigin || [0.0, 0.0, 0.0];
        mouseRayDirection = mouseRayDirection || [0.0, 0.0, 1.0];

        if (viewMatrix) {
            this.setViewMatrix(viewMatrix);
        }

        if (timeStep === 0.0) return;

        this.frameNumber += 1;
        this._updateCurrentGravity(timeStep);

        /*
            the simulation process
            transfer particle velocities to velocity grid
            save this velocity grid

            solve velocity grid for non divergence

            update particle velocities with new velocity grid
            advect particles through the grid velocity field
        */

        //////////////////////////////////////////////////////
        // transfer particle velocities to grid

        var transferToGridDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.velocityTextureWidth, this.velocityTextureHeight)
            .vertexAttribPointer(this.particleVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.transferToGridProgram)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniform3f('u_gridSize', this.gridWidth, this.gridHeight, this.gridDepth)
            .uniformTexture('u_positionTexture', 0, wgl.TEXTURE_2D, this.particlePositionTexture)
            .uniformTexture('u_velocityTexture', 1, wgl.TEXTURE_2D, this.particleVelocityTexture)
            .enable(wgl.BLEND)
            .blendEquation(wgl.FUNC_ADD)
            .blendFuncSeparate(wgl.ONE, wgl.ONE, wgl.ONE, wgl.ONE);

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.weightTexture, 0);

        wgl.clear(
            wgl.createClearState().bindFramebuffer(this.simulationFramebuffer).clearColor(0, 0, 0, 0),
            wgl.COLOR_BUFFER_BIT
        );

        transferToGridDrawState.uniform1i('u_accumulate', 0);

        var SPLAT_DEPTH = 5;

        for (var z = -(SPLAT_DEPTH - 1) / 2; z <= (SPLAT_DEPTH - 1) / 2; ++z) {
            transferToGridDrawState.uniform1f('u_zOffset', z);
            wgl.drawArrays(transferToGridDrawState, wgl.POINTS, 0, this.particlesWidth * this.particlesHeight);
        }

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.tempVelocityTexture, 0);
        wgl.clear(
            wgl.createClearState().bindFramebuffer(this.simulationFramebuffer),
            wgl.COLOR_BUFFER_BIT
        );

        transferToGridDrawState.uniform1i('u_accumulate', 1);

        for (z = -(SPLAT_DEPTH - 1) / 2; z <= (SPLAT_DEPTH - 1) / 2; ++z) {
            transferToGridDrawState.uniform1f('u_zOffset', z);
            wgl.drawArrays(transferToGridDrawState, wgl.POINTS, 0, this.particlesWidth * this.particlesHeight);
        }

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.velocityTexture, 0);

        var normalizeDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.velocityTextureWidth, this.velocityTextureHeight)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.normalizeGridProgram)
            .uniformTexture('u_weightTexture', 0, wgl.TEXTURE_2D, this.weightTexture)
            .uniformTexture('u_accumulatedVelocityTexture', 1, wgl.TEXTURE_2D, this.tempVelocityTexture);

        wgl.drawArrays(normalizeDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        //////////////////////////////////////////////////////
        // mark cells with fluid

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.markerTexture, 0);
        wgl.clear(
            wgl.createClearState().bindFramebuffer(this.simulationFramebuffer),
            wgl.COLOR_BUFFER_BIT
        );

        var markDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.scalarTextureWidth, this.scalarTextureHeight)
            .vertexAttribPointer(this.particleVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.markProgram)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniform3f('u_gridSize', this.gridWidth, this.gridHeight, this.gridDepth)
            .uniformTexture('u_positionTexture', 0, wgl.TEXTURE_2D, this.particlePositionTexture);

        wgl.drawArrays(markDrawState, wgl.POINTS, 0, this.particlesWidth * this.particlesHeight);

        ////////////////////////////////////////////////////
        // save original velocity grid

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.originalVelocityTexture, 0);

        var copyDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.velocityTextureWidth, this.velocityTextureHeight)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.copyProgram)
            .uniformTexture('u_texture', 0, wgl.TEXTURE_2D, this.velocityTexture);

        wgl.drawArrays(copyDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        /////////////////////////////////////////////////////
        // add forces to velocity grid

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.tempVelocityTexture, 0);

        var addForceDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.velocityTextureWidth, this.velocityTextureHeight)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.addForceProgram)
            .uniformTexture('u_velocityTexture', 0, wgl.TEXTURE_2D, this.velocityTexture)
            .uniform1f('u_timeStep', timeStep)
            .uniform3f('u_mouseVelocity', mouseVelocity[0], mouseVelocity[1], mouseVelocity[2])
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniform3f('u_gridSize', this.gridWidth, this.gridHeight, this.gridDepth)
            .uniform3f('u_mouseRayOrigin', mouseRayOrigin[0], mouseRayOrigin[1], mouseRayOrigin[2])
            .uniform3f('u_mouseRayDirection', mouseRayDirection[0], mouseRayDirection[1], mouseRayDirection[2])
            .uniform3f('u_gravity', this.currentGravity[0], this.currentGravity[1], this.currentGravity[2]);

        wgl.drawArrays(addForceDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        swap(this, 'velocityTexture', 'tempVelocityTexture');

        /////////////////////////////////////////////////////
        // enforce boundary velocity conditions

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.tempVelocityTexture, 0);

        var enforceBoundariesDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.velocityTextureWidth, this.velocityTextureHeight)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.enforceBoundariesProgram)
            .uniformTexture('u_velocityTexture', 0, wgl.TEXTURE_2D, this.velocityTexture)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ);

        wgl.drawArrays(enforceBoundariesDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        swap(this, 'velocityTexture', 'tempVelocityTexture');

        /////////////////////////////////////////////////////
        // update velocityTexture for non divergence

        var divergenceDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.scalarTextureWidth, this.scalarTextureHeight)
            .useProgram(this.divergenceProgram)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniformTexture('u_velocityTexture', 0, wgl.TEXTURE_2D, this.velocityTexture)
            .uniformTexture('u_markerTexture', 1, wgl.TEXTURE_2D, this.markerTexture)
            .uniformTexture('u_weightTexture', 2, wgl.TEXTURE_2D, this.weightTexture)
            .uniform1f('u_maxDensity', this.particleDensity)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, false, 0, 0);

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.divergenceTexture, 0);
        wgl.clear(
            wgl.createClearState().bindFramebuffer(this.simulationFramebuffer),
            wgl.COLOR_BUFFER_BIT
        );

        wgl.drawArrays(divergenceDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        var jacobiDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.scalarTextureWidth, this.scalarTextureHeight)
            .useProgram(this.jacobiProgram)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniformTexture('u_divergenceTexture', 1, wgl.TEXTURE_2D, this.divergenceTexture)
            .uniformTexture('u_markerTexture', 2, wgl.TEXTURE_2D, this.markerTexture)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, false, 0, 0);

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.pressureTexture, 0);
        wgl.clear(
            wgl.createClearState().bindFramebuffer(this.simulationFramebuffer),
            wgl.COLOR_BUFFER_BIT
        );

        var PRESSURE_JACOBI_ITERATIONS = 50;
        for (let i = 0; i < PRESSURE_JACOBI_ITERATIONS; ++i) {
            wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.tempSimulationTexture, 0);
            jacobiDrawState.uniformTexture('u_pressureTexture', 0, wgl.TEXTURE_2D, this.pressureTexture);
            wgl.drawArrays(jacobiDrawState, wgl.TRIANGLE_STRIP, 0, 4);
            swap(this, 'pressureTexture', 'tempSimulationTexture');
        }

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.tempVelocityTexture, 0);

        var subtractDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.velocityTextureWidth, this.velocityTextureHeight)
            .useProgram(this.subtractProgram)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniformTexture('u_pressureTexture', 0, wgl.TEXTURE_2D, this.pressureTexture)
            .uniformTexture('u_velocityTexture', 1, wgl.TEXTURE_2D, this.velocityTexture)
            .uniformTexture('u_markerTexture', 2, wgl.TEXTURE_2D, this.markerTexture)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, false, 0, 0);

        wgl.drawArrays(subtractDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        swap(this, 'velocityTexture', 'tempVelocityTexture');

        /////////////////////////////////////////////////////////////
        // transfer velocities back to particles

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.particleVelocityTextureTemp, 0);

        var transferToParticlesDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.particlesWidth, this.particlesHeight)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.transferToParticlesProgram)
            .uniformTexture('u_particlePositionTexture', 0, wgl.TEXTURE_2D, this.particlePositionTexture)
            .uniformTexture('u_particleVelocityTexture', 1, wgl.TEXTURE_2D, this.particleVelocityTexture)
            .uniformTexture('u_gridVelocityTexture', 2, wgl.TEXTURE_2D, this.velocityTexture)
            .uniformTexture('u_originalGridVelocityTexture', 3, wgl.TEXTURE_2D, this.originalVelocityTexture)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniform3f('u_gridSize', this.gridWidth, this.gridHeight, this.gridDepth)
            .uniform1f('u_flipness', this.flipness);

        wgl.drawArrays(transferToParticlesDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        swap(this, 'particleVelocityTextureTemp', 'particleVelocityTexture');

        ///////////////////////////////////////////////
        // advect particle positions with velocity grid using RK2

        wgl.framebufferTexture2D(this.simulationFramebuffer, wgl.FRAMEBUFFER, wgl.COLOR_ATTACHMENT0, wgl.TEXTURE_2D, this.particlePositionTextureTemp, 0);
        wgl.clear(
            wgl.createClearState().bindFramebuffer(this.simulationFramebuffer),
            wgl.COLOR_BUFFER_BIT
        );

        var advectDrawState = wgl.createDrawState()
            .bindFramebuffer(this.simulationFramebuffer)
            .viewport(0, 0, this.particlesWidth, this.particlesHeight)
            .vertexAttribPointer(this.quadVertexBuffer, 0, 2, wgl.FLOAT, wgl.FALSE, 0, 0)
            .useProgram(this.advectProgram)
            .uniformTexture('u_positionsTexture', 0, wgl.TEXTURE_2D, this.particlePositionTexture)
            .uniformTexture('u_randomsTexture', 1, wgl.TEXTURE_2D, this.particleRandomTexture)
            .uniformTexture('u_velocityGrid', 2, wgl.TEXTURE_2D, this.velocityTexture)
            .uniform3f('u_gridResolution', this.gridResolutionX, this.gridResolutionY, this.gridResolutionZ)
            .uniform3f('u_gridSize', this.gridWidth, this.gridHeight, this.gridDepth)
            .uniform1f('u_timeStep', timeStep)
            .uniform1f('u_frameNumber', this.frameNumber)
            .uniform2f('u_particlesResolution', this.particlesWidth, this.particlesHeight);

        wgl.drawArrays(advectDrawState, wgl.TRIANGLE_STRIP, 0, 4);

        swap(this, 'particlePositionTextureTemp', 'particlePositionTexture');
    };

    return Simulator;
}());
