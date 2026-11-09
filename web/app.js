const $ = id => document.getElementById(id);
const [canvas, fpsDisplay, btnConnect] = ['view', 'fps', 'btnConnect'].map($);
const ctx = canvas.getContext('2d');
const [elLatJpeg, elLatAi, elLatServoPan, elLatServoTilt, elLatUsb, elLatCore1, elLatCore0, elStatBandwidth, elStatJitter] =
  ['latJpeg', 'latAi', 'latServoPan', 'latServoTilt', 'latUsb', 'latCore1', 'latCore0', 'statBandwidth', 'statJitter'].map($);

let frameCount = 0;
let lastFpsTime = performance.now();
let lastMeasuredFps = 24;
let bytesThisSec = 0;
let lastRealJpegMs = 0;
let lastRealAiMs = 0;
let lastRealPanUs = 0;
let lastRealTiltUs = 0;
let lastRealUsbMs = 0;
let lastPacketTime = 0;
let lastFrameGapMs = 0;

// Session statistics (real measured data for benchmarking/report)
let sessAiCount = 0, sessAiTotal = 0, sessAiMin = Infinity, sessAiMax = 0;
let sessTrackFrames = 0, sessTotalFrames = 0, sessScoreSum = 0;

function resetSessionStats() {
  sessAiCount = 0; sessAiTotal = 0; sessAiMin = Infinity; sessAiMax = 0;
  sessTrackFrames = 0; sessTotalFrames = 0; sessScoreSum = 0;
}

let reader = null;
let currentPort = null;
let streamRunning = false;
let buffer = new Uint8Array(0);

// Hardware BBox from ESP32-S3 (Sony Real-Time Tracking State)
let currentBBox = { status: 0, x: 120, y: 120, w: 0, h: 0, lastSeen: 0 };

function appendBuffer(b1, b2) {
  const merged = new Uint8Array(b1.length + b2.length);
  merged.set(b1);
  merged.set(b2, b1.length);
  return merged;
}

function trackFps() {
  frameCount++;
  const now = performance.now();
  if (now - lastFpsTime >= 1000) {
    lastMeasuredFps = Math.round((frameCount * 1000) / (now - lastFpsTime));
    fpsDisplay.textContent = `${lastMeasuredFps} FPS`;
    frameCount = 0;
    lastFpsTime = now;
  }
}

const teleEls = [elLatJpeg, elLatAi, elLatServoPan, elLatServoTilt, elLatUsb, elLatCore1, elLatCore0, elStatBandwidth, elStatJitter];

// Telemetry Update with 1500ms Timeout Fallback ('__')
setInterval(() => {
  const now = performance.now();
  if (!streamRunning || lastPacketTime === 0 || (now - lastPacketTime > 1500)) {
    teleEls.forEach(el => { if (el) el.textContent = '__'; });
    fpsDisplay.textContent = '0 FPS';
    return;
  }

  const mbps = ((bytesThisSec * 8) / 1000000).toFixed(2);
  bytesThisSec = 0;
  if (elLatJpeg) elLatJpeg.textContent = `${lastRealJpegMs.toFixed(1)} ms`;
  if (elLatAi) elLatAi.textContent = `${lastRealAiMs.toFixed(1)} ms`;
  if (elLatServoPan) elLatServoPan.textContent = `${lastRealPanUs.toFixed(1)} µs`;
  if (elLatServoTilt) elLatServoTilt.textContent = `${lastRealTiltUs.toFixed(1)} µs`;
  if (elLatUsb) elLatUsb.textContent = `${lastRealUsbMs.toFixed(1)} ms`;
  if (elLatCore1) elLatCore1.textContent = `${(lastRealJpegMs + lastRealAiMs).toFixed(1)} ms`;
  if (elLatCore0) elLatCore0.textContent = `${(lastRealUsbMs + (lastRealPanUs + lastRealTiltUs) / 1000).toFixed(2)} ms`;
  if (elStatBandwidth) elStatBandwidth.textContent = `${mbps} Mbps`;
  if (elStatJitter) elStatJitter.textContent = `${lastFrameGapMs.toFixed(1)} ms`;
}, 500);

// Session statistics panel update (real measured values)
setInterval(() => {
  if (!streamRunning || lastPacketTime === 0 || (performance.now() - lastPacketTime > 1500)) {
    ['statAiAvg', 'statAiMinMax', 'statTrackRate'].forEach(id => {
      const el = $(id); if (el) el.textContent = '__';
    });
    return;
  }
  if (sessAiCount > 0) {
    const el = $('statAiAvg'); if (el) el.textContent = `${(sessAiTotal / sessAiCount).toFixed(1)} ms`;
    const el2 = $('statAiMinMax'); if (el2) el2.textContent = `${sessAiMin.toFixed(1)} / ${sessAiMax.toFixed(1)} ms`;
  }
  if (sessTotalFrames > 0) {
    const el3 = $('statTrackRate'); if (el3) el3.textContent = `${((sessTrackFrames / sessTotalFrames) * 100).toFixed(1)} %`;
  }
}, 1000);

function drawBoundingBox() {
  const isVisible = currentBBox.status !== 0 && (performance.now() - currentBBox.lastSeen < 250);
  if (isVisible && currentBBox.w > 0 && currentBBox.h > 0) {
    const { x, y, w, h } = currentBBox;
    const color = '#00FF66';
    const label = 'FACE';

    ctx.strokeStyle = color;
    ctx.lineWidth = 0.8;
    ctx.strokeRect(x + 0.5, y + 0.5, w, h);

    ctx.fillStyle = color;
    ctx.font = '500 8.5px monospace';
    ctx.textAlign = 'center';
    const tagY = y > 13 ? y - 3 : y + 11;
    ctx.fillText(label, x + Math.round(w / 2), tagY);
  }
}

function renderJpeg(frameBytes) {
  const now = performance.now();
  if (lastPacketTime > 0) {
    lastFrameGapMs = now - lastPacketTime; // real inter-frame interval (jitter)
  }
  lastPacketTime = now;
  createImageBitmap(new Blob([frameBytes], { type: 'image/jpeg' }))
    .then(bitmap => {
      ctx.drawImage(bitmap, 0, 0, 240, 240);
      bitmap.close();
      drawBoundingBox();
      trackFps();
    })
    .catch(() => {});
}

function parseStream() {
  while (buffer.length >= 6) {
    // Extended Protocol: [0xAA, 0xBB, status, x(2B), y(2B), w(2B), h(2B), t_jpeg(2B), t_ai(2B), t_pan(2B), t_tilt(2B), payloadLen(4B), t_usb(2B), heapInt(2B), heapPsram(2B)] = 29 Bytes Header
    if (buffer[0] === 0xAA && buffer[1] === 0xBB) {
      if (buffer.length < 29) break;

      const payloadLen = ((buffer[19] << 24) | (buffer[20] << 16) | (buffer[21] << 8) | buffer[22]) >>> 0;
      if (payloadLen > 65536 || payloadLen < 100) {
        buffer = buffer.slice(2);
        continue;
      }

      if (buffer.length >= 29 + payloadLen) {
        const status = buffer[2];
        const rawJpeg = (buffer[11] << 8) | buffer[12];
        const rawAi = (buffer[13] << 8) | buffer[14];
        const rawPan = (buffer[15] << 8) | buffer[16];
        const rawTilt = (buffer[17] << 8) | buffer[18];
        const rawUsb = (buffer[23] << 8) | buffer[24];
        const heapIntKb = (buffer[25] << 8) | buffer[26];
        const heapPsramKb = (buffer[27] << 8) | buffer[28];

        if (rawJpeg > 0 && rawJpeg < 1000) lastRealJpegMs = rawJpeg / 10.0;
        if (rawAi > 0 && rawAi < 2000) lastRealAiMs = rawAi / 10.0;
        if (rawPan > 0 && rawPan < 10000) lastRealPanUs = rawPan / 10.0;
        if (rawTilt > 0 && rawTilt < 10000) lastRealTiltUs = rawTilt / 10.0;
        if (rawUsb > 0 && rawUsb < 1000) lastRealUsbMs = rawUsb / 10.0;

        const elH1 = $('statHeapInt'); if (elH1) elH1.textContent = `${heapIntKb} KB trống (đo runtime)`;
        const elH2 = $('statHeapPsram'); if (elH2) elH2.textContent = `${heapPsramKb} KB / 8192 KB trống`;

        // Session statistics
        if (rawAi > 0 && rawAi < 2000) {
          const aiMs = rawAi / 10.0;
          sessAiCount++; sessAiTotal += aiMs;
          if (aiMs < sessAiMin) sessAiMin = aiMs;
          if (aiMs > sessAiMax) sessAiMax = aiMs;
        }
        sessTotalFrames++;
        if (status === 0x01 || status === 0x02) sessTrackFrames++;

        if (status === 0x01 || status === 0x02) {
          currentBBox = {
            status: status,
            x: (buffer[3] << 8) | buffer[4],
            y: (buffer[5] << 8) | buffer[6],
            w: (buffer[7] << 8) | buffer[8],
            h: (buffer[9] << 8) | buffer[10],
            lastSeen: performance.now()
          };
        } else {
          currentBBox.status = 0;
        }

        const frameData = buffer.slice(29, 29 + payloadLen);
        buffer = buffer.slice(29 + payloadLen);
        renderJpeg(frameData);
        continue;
      }
      break;
    }

    // 2. Direct JPEG Marker Fallback [0xFF, 0xD8]
    if (buffer[0] === 0xFF && buffer[1] === 0xD8) {
      let eoi = -1;
      for (let i = 2; i < buffer.length - 1; i++) {
        if (buffer[i] === 0xFF && buffer[i + 1] === 0xD9) {
          eoi = i + 2;
          break;
        }
      }
      if (eoi !== -1) {
        const frameData = buffer.slice(0, eoi);
        buffer = buffer.slice(eoi);
        renderJpeg(frameData);
        continue;
      }
      break;
    }

    let nextIdx = -1;
    for (let i = 1; i < buffer.length - 1; i++) {
      if ((buffer[i] === 0xAA && buffer[i + 1] === 0xBB) || (buffer[i] === 0xFF && buffer[i + 1] === 0xD8)) {
        nextIdx = i;
        break;
      }
    }
    if (nextIdx !== -1) {
      buffer = buffer.slice(nextIdx);
    } else {
      buffer = buffer.slice(buffer.length - 1);
      break;
    }
  }
}

async function closeStream() {
  streamRunning = false;
  if (reader) {
    try {
      await reader.cancel();
      reader.releaseLock();
    } catch (e) {}
    reader = null;
  }
  if (currentPort) {
    try {
      await currentPort.close();
    } catch (e) {}
    currentPort = null;
  }
  btnConnect.textContent = 'CHỌN CỔNG COM';
  btnConnect.style.borderColor = '#000000';
  fpsDisplay.textContent = '0 FPS';
  lastPacketTime = 0;
}

async function startStream(forceSelect = false) {
  if (streamRunning && forceSelect) {
    await closeStream();
  }

  try {
    let port = null;
    if (forceSelect || !currentPort) {
      port = await navigator.serial.requestPort();
    } else {
      port = currentPort;
    }

    await port.open({ baudRate: 2000000 });
    currentPort = port;
    streamRunning = true;
    resetSessionStats();
    frameCount = 0;
    lastFpsTime = performance.now();
    lastRealJpegMs = 0; lastRealAiMs = 0; lastRealPanUs = 0; lastRealTiltUs = 0; lastRealUsbMs = 0;
    reader = currentPort.readable.getReader();

    btnConnect.textContent = 'NGẮT KẾT NỐI';
    btnConnect.style.borderColor = '#00aa55';

    while (streamRunning) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value) {
        bytesThisSec += value.length;
        buffer = appendBuffer(buffer, value);
        parseStream();
      }
    }
  } catch (err) {
    await closeStream();
  }
}

btnConnect.addEventListener('click', async () => {
  streamRunning ? await closeStream() : await startStream(true);
});
