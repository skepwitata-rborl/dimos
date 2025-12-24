import * as THREE from 'three';

const container = document.getElementById('container');
const vrButton = document.getElementById('vr-button');
const errorDiv = document.getElementById('error');
const wsStatus = document.getElementById('ws-status');

let scene;
let camera;
let renderer;
let referenceSpace;

const controllers = [];
const infoPanels = [];

const BUTTON_PRESS_THRESHOLD = 0.55;
const BUTTON_TOUCH_THRESHOLD = 0.05;
const SEND_INTERVAL_MS = 1000 / 90;

const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
const WS_URL = `${wsProtocol}//${window.location.host}/ws`;

class WebSocketManager {
    constructor(url, statusElement) {
        this.url = url;
        this.statusElement = statusElement;
        this.ws = null;
        this.connected = false;
        this.reconnectDelay = 1000;
        this.maxReconnectDelay = 10000;
        this.connectionTimer = null;
        this.retryTimer = null;
        this.connect();
        window.addEventListener('beforeunload', () => this.shutdown());
    }

    connect() {
        if (this.ws && (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING)) {
            return;
        }

        try {
            this._updateStatus('WebSocket: Connecting…', '#ff0');
            this.ws = new WebSocket(this.url);
            this.ws.binaryType = 'arraybuffer';

            this.connectionTimer = setTimeout(() => {
                if (!this.connected && this.ws) {
                    console.warn('WebSocket connection timed out, forcing reconnect');
                    try {
                        this.ws.close();
                    } catch (err) {
                        console.warn('Failed to close timed out WebSocket', err);
                    }
                }
            }, 4000);

            this.ws.onopen = () => {
                this.connected = true;
                this.reconnectDelay = 1000;
                this._clearConnectionTimer();
                this._updateStatus('WebSocket: Connected', '#0f0');
                console.log('[ws] connected');
            };

            this.ws.onclose = (event) => {
                const wasConnected = this.connected;
                this.connected = false;
                this._clearConnectionTimer();
                this._updateStatus('WebSocket: Disconnected – retrying…', '#f80');
                console.warn('[ws] closed', event.reason || event.code);
                this._scheduleReconnect(wasConnected);
            };

            this.ws.onerror = (event) => {
                console.error('[ws] error', event);
                this._updateStatus('WebSocket: Error – retrying', '#f00');
            };
        } catch (error) {
            console.error('[ws] connection exception', error);
            this._updateStatus('WebSocket: Failed to connect', '#f00');
            this._scheduleReconnect(false);
        }
    }

    _scheduleReconnect(wasConnected) {
        if (this.retryTimer) {
            return;
        }

        this.retryTimer = setTimeout(() => {
            this.retryTimer = null;
            this.connect();
        }, this.reconnectDelay);

        if (wasConnected) {
            this.reconnectDelay = 1000;
        } else {
            this.reconnectDelay = Math.min(this.reconnectDelay * 1.5, this.maxReconnectDelay);
        }
    }

    _updateStatus(text, color) {
        if (!this.statusElement) {
            return;
        }
        this.statusElement.textContent = text;
        this.statusElement.style.color = color;
    }

    _clearConnectionTimer() {
        if (this.connectionTimer) {
            clearTimeout(this.connectionTimer);
            this.connectionTimer = null;
        }
    }

    isReady() {
        return Boolean(this.ws && this.connected && this.ws.readyState === WebSocket.OPEN);
    }

    send(payload) {
        if (!this.isReady()) {
            return false;
        }

        try {
            this.ws.send(payload);
            return true;
        } catch (error) {
            console.error('[ws] send failed', error);
            this._updateStatus('WebSocket: Send failed – retrying', '#f00');
            try {
                this.ws.close();
            } catch (closeError) {
                console.error('[ws] failed to close after send error', closeError);
            }
            return false;
        }
    }

    shutdown() {
        this.connected = false;
        this._clearConnectionTimer();
        if (this.retryTimer) {
            clearTimeout(this.retryTimer);
            this.retryTimer = null;
        }
        if (this.ws) {
            try {
                this.ws.close();
            } catch (error) {
                console.warn('[ws] shutdown close failed', error);
            }
            this.ws = null;
        }
    }
}

const wsManager = new WebSocketManager(WS_URL, wsStatus);

if ('xr' in navigator) {
    navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
        if (!supported) {
            vrButton.disabled = true;
            vrButton.textContent = 'WebXR not supported';
            showError('WebXR VR is not supported on this device');
        }
    }).catch((error) => {
        console.error('WebXR capability check failed', error);
        showError('Unable to verify WebXR support');
    });
} else {
    vrButton.disabled = true;
    vrButton.textContent = 'WebXR unavailable';
    showError('WebXR is not available in this browser');
}

function showError(message) {
    if (!errorDiv) {
        console.error('UI error container missing');
        return;
    }
    errorDiv.textContent = message;
    errorDiv.style.display = 'block';
}

function init() {
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x000000);

    camera = new THREE.PerspectiveCamera(
        75,
        window.innerWidth / window.innerHeight,
        0.1,
        1000
    );
    camera.position.set(0, 1.6, 3);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.xr.enabled = true;
    container.appendChild(renderer.domElement);

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 10, 5);
    scene.add(directionalLight);

    const gridHelper = new THREE.GridHelper(10, 10, 0x00ff00, 0x003300);
    gridHelper.position.y = 0;
    scene.add(gridHelper);

    setupControllers();
    createInfoPanels();

    window.addEventListener('resize', onWindowResize);

    renderer.setAnimationLoop(animate);
}

const controllerState = [
    { connected: false, hand: null },
    { connected: false, hand: null }
];

function createInfoPanels() {
    const positions = [
        new THREE.Vector3(-1, 1.6, -1.5),
        new THREE.Vector3(1, 1.6, -1.5)
    ];

    const colors = [0xff0066, 0x0066ff];

    for (let i = 0; i < 2; i++) {
        const canvas = document.createElement('canvas');
        canvas.width = 512;
        canvas.height = 512;

        const texture = new THREE.CanvasTexture(canvas);
        texture.needsUpdate = true;

        const material = new THREE.MeshBasicMaterial({
            map: texture,
            transparent: true,
            side: THREE.DoubleSide
        });

        const geometry = new THREE.PlaneGeometry(0.8, 0.8);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.copy(positions[i]);

        scene.add(mesh);

        infoPanels.push({
            canvas,
            texture,
            mesh,
            color: colors[i]
        });
    }
}

function updateInfoPanel(index, state) {
    const panel = infoPanels[index];
    if (!panel) {
        return;
    }

    const ctx = panel.canvas.getContext('2d');
    ctx.fillStyle = 'rgba(0, 0, 0, 0.9)';
    ctx.fillRect(0, 0, panel.canvas.width, panel.canvas.height);

    ctx.fillStyle = '#00ff00';
    ctx.font = 'bold 20px Courier New';
    ctx.textAlign = 'left';

    let y = 30;
    const lineHeight = 22;

    const handedness = index === 0 ? 'LEFT' : 'RIGHT';
    ctx.fillStyle = `#${panel.color.toString(16).padStart(6, '0')}`;
    ctx.fillText(`${handedness} CONTROLLER`, 20, y);
    y += lineHeight * 1.5;

    ctx.fillStyle = '#00ff00';
    ctx.fillText(`Connected: ${state.connected ? 'YES' : 'NO'}`, 20, y);
    y += lineHeight * 1.5;

    if (!state.connected) {
        ctx.fillStyle = '#ffff00';
        ctx.fillText('Waiting for controller...', 20, y);
        panel.texture.needsUpdate = true;
        return;
    }

    ctx.fillText('POSITION:', 20, y);
    y += lineHeight;
    ctx.fillText(`  X: ${state.position.x.toFixed(3)}`, 20, y);
    y += lineHeight;
    ctx.fillText(`  Y: ${state.position.y.toFixed(3)}`, 20, y);
    y += lineHeight;
    ctx.fillText(`  Z: ${state.position.z.toFixed(3)}`, 20, y);
    y += lineHeight * 1.5;

    ctx.fillText('ROTATION (deg):', 20, y);
    y += lineHeight;
    ctx.fillText(`  X: ${state.rotationEuler.x.toFixed(1)}`, 20, y);
    y += lineHeight;
    ctx.fillText(`  Y: ${state.rotationEuler.y.toFixed(1)}`, 20, y);
    y += lineHeight;
    ctx.fillText(`  Z: ${state.rotationEuler.z.toFixed(1)}`, 20, y);
    y += lineHeight * 1.5;

    ctx.fillText('BUTTONS:', 20, y);
    y += lineHeight;

    const labels = ['Trigger', 'Grip', 'Menu', 'Stick', 'A/X', 'B/Y'];
    state.buttonValues.forEach((value, i) => {
        const label = labels[i] || `Btn${i}`;
        const status = value > BUTTON_PRESS_THRESHOLD
            ? 'PRESSED'
            : value > BUTTON_TOUCH_THRESHOLD
                ? 'TOUCH'
                : 'OFF';
        const color = value > BUTTON_PRESS_THRESHOLD
            ? '#00ff00'
            : value > BUTTON_TOUCH_THRESHOLD
                ? '#ffff00'
                : '#666666';
        ctx.fillStyle = color;
        ctx.fillText(`  ${label}: ${status} (${value.toFixed(2)})`, 20, y);
        ctx.fillStyle = '#00ff00';
        y += lineHeight;
    });

    y += lineHeight * 0.5;
    ctx.fillText('THUMBSTICK:', 20, y);
    y += lineHeight;
    ctx.fillText(`  X: ${state.axes[0].toFixed(3)}`, 20, y);
    y += lineHeight;
    ctx.fillText(`  Y: ${state.axes[1].toFixed(3)}`, 20, y);

    panel.texture.needsUpdate = true;
}

function setupControllers() {
    for (let i = 0; i < 2; i++) {
        const controller = renderer.xr.getController(i);
        controller.userData.index = i;
        scene.add(controller);

        const geometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, 0, -1)
        ]);
        const line = new THREE.Line(geometry);
        line.name = 'line';
        line.scale.z = 5;
        line.material.color.setHex(i === 0 ? 0xff0066 : 0x0066ff);
        controller.add(line);

        const controllerGrip = renderer.xr.getControllerGrip(i);
        const gripGeometry = new THREE.SphereGeometry(0.03, 16, 16);
        const gripMaterial = new THREE.MeshStandardMaterial({
            color: i === 0 ? 0xff0066 : 0x0066ff,
            metalness: 0.7,
            roughness: 0.3
        });
        const gripMesh = new THREE.Mesh(gripGeometry, gripMaterial);
        controllerGrip.add(gripMesh);
        scene.add(controllerGrip);

        controllers.push({
            controller,
            grip: controllerGrip
        });
    }
}

let lastInputSourceCount = 0;

const defaultControllerState = () => ({
    connected: false,
    position: { x: 0, y: 0, z: 0 },
    rotationEuler: { x: 0, y: 0, z: 0 },
    quaternion: { x: 0, y: 0, z: 0, w: 1 },
    buttonValues: [0, 0, 0, 0, 0, 0],
    axes: [0, 0]
});

function getControllerData(frame) {
    if (!frame) {
        return null;
    }

    const session = frame.session;
    if (!session) {
        return null;
    }

    referenceSpace = renderer.xr.getReferenceSpace();
    if (!referenceSpace) {
        return null;
    }

    let leftAssigned = false;
    let rightAssigned = false;

    for (const source of session.inputSources) {
        if (source.handedness === 'left' || source.handedness === 'right') {
            const index = source.handedness === 'left' ? 0 : 1;
            if (!controllerState[index].connected) {
                controllerState[index].connected = true;
                controllerState[index].hand = source.handedness;
                console.log(`✓ Controller detected: ${source.handedness}`);
            }
            if (index === 0) leftAssigned = true;
            if (index === 1) rightAssigned = true;
        } else if (source.gamepad && source.gamepad.buttons && source.gamepad.buttons.length > 0) {
            const index = leftAssigned ? 1 : 0;
            if (!controllerState[index].connected) {
                controllerState[index].connected = true;
                controllerState[index].hand = index === 0 ? 'left' : 'right';
                console.log(`✓ Fallback assigned: ${controllerState[index].hand}`);
            }
            if (index === 0) leftAssigned = true;
            if (index === 1) rightAssigned = true;
        }
    }

    if (leftAssigned !== rightAssigned) {
        const missingIndex = leftAssigned ? 1 : 0;
        if (!controllerState[missingIndex].connected && session.inputSources.length > 1) {
            controllerState[missingIndex].connected = true;
            controllerState[missingIndex].hand = missingIndex === 0 ? 'left' : 'right';
            console.log(`✓ Auto-assigned missing: ${controllerState[missingIndex].hand}`);
        }
    }

    if (session.inputSources.length !== lastInputSourceCount) {
        console.log(`InputSources: ${session.inputSources.length} (L:${controllerState[0].connected?'✓':'✗'}, R:${controllerState[1].connected?'✓':'✗'})`);
        lastInputSourceCount = session.inputSources.length;
    }

    const results = [defaultControllerState(), defaultControllerState()];

    for (const source of session.inputSources) {
        const handedness = source.handedness;
        if (handedness !== 'left' && handedness !== 'right') continue;

        const index = handedness === 'left' ? 0 : 1;
        const state = results[index];
        state.connected = true;

        if (source.gripSpace) {
            const gripPose = frame.getPose(source.gripSpace, referenceSpace);
            if (gripPose) {
                const pos = gripPose.transform.position;
                state.position = { x: pos.x, y: pos.y, z: pos.z };

                const quat = gripPose.transform.orientation;
                state.quaternion = { x: quat.x, y: quat.y, z: quat.z, w: quat.w };

                const threeQuat = new THREE.Quaternion(quat.x, quat.y, quat.z, quat.w);
                const euler = new THREE.Euler().setFromQuaternion(threeQuat, 'XYZ');
                state.rotationEuler = {
                    x: THREE.MathUtils.radToDeg(euler.x),
                    y: THREE.MathUtils.radToDeg(euler.y),
                    z: THREE.MathUtils.radToDeg(euler.z)
                };
            }
        }

        if (source.gamepad) {
            state.buttonValues = source.gamepad.buttons.map((button) => button?.value ?? 0);
            const axes = source.gamepad.axes || [];
            state.axes = [
                typeof axes[0] === 'number' ? axes[0] : 0,
                typeof axes[1] === 'number' ? axes[1] : 0
            ];
        }
    }

    return results;
}

function toButtonState(buttonValues, index) {
    const value = buttonValues[index] ?? 0;
    return {
        value,
        pressed: value >= BUTTON_PRESS_THRESHOLD,
        touched: value >= BUTTON_TOUCH_THRESHOLD
    };
}

function formatControllerPayload(state) {
    if (!state || !state.connected) {
        return null;
    }

    return {
        connected: true,
        position: [state.position.x, state.position.y, state.position.z],
        rotation: [
            state.quaternion.x,
            state.quaternion.y,
            state.quaternion.z,
            state.quaternion.w
        ],
        rotation_euler: [
            state.rotationEuler.x,
            state.rotationEuler.y,
            state.rotationEuler.z
        ],
        buttons: {
            trigger: toButtonState(state.buttonValues, 0),
            grip: toButtonState(state.buttonValues, 1),
            menu: toButtonState(state.buttonValues, 2),
            thumbstick: toButtonState(state.buttonValues, 3),
            x_or_a: toButtonState(state.buttonValues, 4),
            y_or_b: toButtonState(state.buttonValues, 5)
        },
        button_values: state.buttonValues,
        axes: {
            thumbstick_x: state.axes[0],
            thumbstick_y: state.axes[1]
        }
    };
}

let lastSentTimestamp = 0;

function streamToServer(controllerStates, timestampMs) {
    if (!wsManager.isReady()) {
        return;
    }

    const payload = {
        timestamp: timestampMs / 1000,
        left: formatControllerPayload(controllerStates[0]),
        right: formatControllerPayload(controllerStates[1])
    };

    if (!payload.left && !payload.right) {
        return;
    }

    try {
        const serialized = JSON.stringify(payload);
        wsManager.send(serialized);
    } catch (error) {
        console.error('Error serializing frame', error);
    }
}

function animate(_time, frame) {
    const controllerStates = getControllerData(frame);
    if (controllerStates) {
        controllerStates.forEach((state, index) => {
            updateInfoPanel(index, state);
        });

        const now = performance.now();
        if (now - lastSentTimestamp >= SEND_INTERVAL_MS) {
            streamToServer(controllerStates, now);
            lastSentTimestamp = now;
        }
    }

    renderer.render(scene, camera);
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

async function enterVR() {
    try {
        const currentSession = renderer.xr.getSession();
        if (currentSession) {
            await currentSession.end();
            await new Promise((resolve) => setTimeout(resolve, 100));
        }

        const sessionInit = {
            optionalFeatures: ['local-floor', 'bounded-floor', 'hand-tracking']
        };

        const session = await navigator.xr.requestSession('immersive-vr', sessionInit);
        console.log('XR Session created');

        session.addEventListener('inputsourceschange', () => {
            console.log('inputsourceschange:', session.inputSources.length);
        });

        await renderer.xr.setSession(session);
        console.log('XR Session set on renderer');

        document.getElementById('ui-overlay').style.display = 'none';
        document.getElementById('vr-button-container').style.display = 'none';

        session.addEventListener('end', () => {
            console.log('XR Session ended');
            document.getElementById('ui-overlay').style.display = 'block';
            document.getElementById('vr-button-container').style.display = 'flex';
        });
    } catch (err) {
        showError('Error entering VR: ' + err.message);
        console.error(err);
    }
}

vrButton.addEventListener('click', enterVR);

init();
