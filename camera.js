'use strict';

var Camera = (function () {
    var SENSITIVITY = 0.005;
    var PINCH_SENSITIVITY = 0.1; // How fast 2-finger pinch zooms

    var MIN_DISTANCE = 25.0;
    var MAX_DISTANCE = 60.0;

    function Camera (element, orbitPoint) {
        this.element = element;
        this.distance = 40.0;
        this.orbitPoint = orbitPoint;

        this.azimuth = 0.0;
        this.elevation = 0.25;

        this.minElevation = -Math.PI / 4;
        this.maxElevation = Math.PI / 4;

        this.currentMouseX = 0;
        this.currentMouseY = 0;

        this.lastMouseX = 0;
        this.lastMouseY = 0;

        this.mouseDown = false;
        this.lastPinchDistance = null;

        this.viewMatrix = new Float32Array(16);

        this.recomputeViewMatrix();

        // Desktop scroll wheel zoom
        element.addEventListener('wheel', (function (event) {
            var scrollDelta = event.deltaY;
            this.distance += ((scrollDelta > 0) ? 1 : -1) * 2.0;

            if (this.distance < MIN_DISTANCE) this.distance = MIN_DISTANCE;
            if (this.distance > MAX_DISTANCE) this.distance = MAX_DISTANCE;

            this.recomputeViewMatrix();
        }).bind(this));
    }

    Camera.prototype.recomputeViewMatrix = function () {
        var xRotationMatrix = new Float32Array(16),
            yRotationMatrix = new Float32Array(16),
            distanceTranslationMatrix = Utilities.makeIdentityMatrix(new Float32Array(16)),
            orbitTranslationMatrix = Utilities.makeIdentityMatrix(new Float32Array(16));

        Utilities.makeIdentityMatrix(this.viewMatrix);

        Utilities.makeXRotationMatrix(xRotationMatrix, this.elevation);
        Utilities.makeYRotationMatrix(yRotationMatrix, this.azimuth);
        
        distanceTranslationMatrix[14] = -this.distance;
        orbitTranslationMatrix[12] = -this.orbitPoint[0];
        orbitTranslationMatrix[13] = -this.orbitPoint[1];
        orbitTranslationMatrix[14] = -this.orbitPoint[2];

        Utilities.premultiplyMatrix(this.viewMatrix, this.viewMatrix, orbitTranslationMatrix);
        Utilities.premultiplyMatrix(this.viewMatrix, this.viewMatrix, yRotationMatrix);
        Utilities.premultiplyMatrix(this.viewMatrix, this.viewMatrix, xRotationMatrix);
        Utilities.premultiplyMatrix(this.viewMatrix, this.viewMatrix, distanceTranslationMatrix);
    };

    Camera.prototype.getPosition = function () {
        var position = [
            this.distance * Math.sin(Math.PI / 2 - this.elevation) * Math.sin(-this.azimuth) + this.orbitPoint[0],
            this.distance * Math.cos(Math.PI / 2 - this.elevation) + this.orbitPoint[1],
            this.distance * Math.sin(Math.PI / 2 - this.elevation) * Math.cos(-this.azimuth) + this.orbitPoint[2]
        ];

        return position;
    };

    Camera.prototype.isMouseDown = function () {
        return this.mouseDown;
    };

    Camera.prototype.getViewMatrix = function () {
        return this.viewMatrix;
    };

    Camera.prototype.setBounds = function (minElevation, maxElevation) {
        this.minElevation = minElevation;
        this.maxElevation = maxElevation;

        if (this.elevation > this.maxElevation) this.elevation = this.maxElevation;
        if (this.elevation < this.minElevation) this.elevation = this.minElevation;

        this.recomputeViewMatrix();
    };

    // --- Desktop Mouse Handlers ---

    Camera.prototype.onMouseDown = function (event) {
        event.preventDefault();
        var pos = Utilities.getMousePosition(event, this.element);
        this.mouseDown = true;
        this.lastMouseX = pos.x;
        this.lastMouseY = pos.y;
    };

    Camera.prototype.onMouseUp = function (event) {
        event.preventDefault();
        this.mouseDown = false;
    };

    Camera.prototype.onMouseMove = function (event) {
        event.preventDefault();
        var pos = Utilities.getMousePosition(event, this.element);

        if (this.mouseDown) {
            this.currentMouseX = pos.x;
            this.currentMouseY = pos.y;

            var deltaAzimuth = (this.currentMouseX - this.lastMouseX) * SENSITIVITY;
            var deltaElevation = (this.currentMouseY - this.lastMouseY) * SENSITIVITY;

            this.azimuth += deltaAzimuth;
            this.elevation += deltaElevation;

            if (this.elevation > this.maxElevation) this.elevation = this.maxElevation;
            if (this.elevation < this.minElevation) this.elevation = this.minElevation;

            this.recomputeViewMatrix();

            this.lastMouseX = this.currentMouseX;
            this.lastMouseY = this.currentMouseY;
        }
    };

    // --- Mobile Touch Handlers ---

    Camera.prototype.onTouchStart = function (event) {
        event.preventDefault(); // Prevents page scrolling
        var rect = this.element.getBoundingClientRect();

        if (event.touches.length === 1) {
            var touch = event.touches[0];
            this.mouseDown = true;
            this.lastMouseX = touch.clientX - rect.left;
            this.lastMouseY = touch.clientY - rect.top;
            this.lastPinchDistance = null;
        } else if (event.touches.length === 2) {
            this.mouseDown = false; // Don't orbit while zooming
            var dx = event.touches[0].clientX - event.touches[1].clientX;
            var dy = event.touches[0].clientY - event.touches[1].clientY;
            this.lastPinchDistance = Math.sqrt(dx * dx + dy * dy);
        }
    };

    Camera.prototype.onTouchMove = function (event) {
        event.preventDefault();
        var rect = this.element.getBoundingClientRect();

        if (event.touches.length === 1 && this.mouseDown) {
            var touch = event.touches[0];
            this.currentMouseX = touch.clientX - rect.left;
            this.currentMouseY = touch.clientY - rect.top;

            var deltaAzimuth = (this.currentMouseX - this.lastMouseX) * SENSITIVITY;
            var deltaElevation = (this.currentMouseY - this.lastMouseY) * SENSITIVITY;

            this.azimuth += deltaAzimuth;
            this.elevation += deltaElevation;

            if (this.elevation > this.maxElevation) this.elevation = this.maxElevation;
            if (this.elevation < this.minElevation) this.elevation = this.minElevation;

            this.recomputeViewMatrix();

            this.lastMouseX = this.currentMouseX;
            this.lastMouseY = this.currentMouseY;
        } else if (event.touches.length === 2) {
            var dx = event.touches[0].clientX - event.touches[1].clientX;
            var dy = event.touches[0].clientY - event.touches[1].clientY;
            var dist = Math.sqrt(dx * dx + dy * dy);

            if (this.lastPinchDistance !== null) {
                var delta = this.lastPinchDistance - dist;
                this.distance += delta * PINCH_SENSITIVITY;

                if (this.distance < MIN_DISTANCE) this.distance = MIN_DISTANCE;
                if (this.distance > MAX_DISTANCE) this.distance = MAX_DISTANCE;

                this.recomputeViewMatrix();
            }
            this.lastPinchDistance = dist;
        }
    };

    Camera.prototype.onTouchEnd = function (event) {
        event.preventDefault();
        var rect = this.element.getBoundingClientRect();

        if (event.touches.length === 0) {
            this.mouseDown = false;
            this.lastPinchDistance = null;
        } else if (event.touches.length === 1) {
            // Dropping from 2 fingers to 1 finger. Reset orbit origin to avoid jumping.
            var touch = event.touches[0];
            this.mouseDown = true;
            this.lastMouseX = touch.clientX - rect.left;
            this.lastMouseY = touch.clientY - rect.top;
            this.lastPinchDistance = null;
        }
    };

    return Camera;
}());
