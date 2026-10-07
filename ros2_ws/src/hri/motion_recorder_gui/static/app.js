(function () {
  const state = { ws: null, requestId: 0, pending: new Map(), status: null, recordingsKey: "" };

  // ---------------------------------------------------------------- transport

  function connect() {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    state.ws = new WebSocket(`${proto}://${location.host}/ws`);
    state.ws.onopen = () => {
      setStatus("connected");
      document.getElementById("conn-lost").hidden = true;
    };
    state.ws.onclose = () => {
      setStatus("disconnected, retrying…");
      document.getElementById("conn-lost").hidden = false;
      for (const { reject } of state.pending.values()) reject(new Error("disconnected"));
      state.pending.clear();
      setTimeout(connect, 1000);
    };
    state.ws.onerror = () => setStatus("connection error");
    state.ws.onmessage = (event) => handleMessage(JSON.parse(event.data));
  }

  function setStatus(text) {
    document.getElementById("status").textContent = text;
  }

  function handleMessage(message) {
    if (message.type === "status") {
      render(message.status);
    } else if (message.type === "ack" && state.pending.has(message.request_id)) {
      const { resolve, reject } = state.pending.get(message.request_id);
      state.pending.delete(message.request_id);
      if (message.ok) resolve(message);
      else reject(new Error(message.error || message.message || "request_failed"));
    }
  }

  function sendCommand(action, payload) {
    return new Promise((resolve, reject) => {
      if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
        reject(new Error("not_connected"));
        return;
      }
      const requestId = String(++state.requestId);
      state.pending.set(requestId, { resolve, reject });
      state.ws.send(JSON.stringify(Object.assign({ action, request_id: requestId }, payload || {})));
    });
  }

  async function run(action, payload, label) {
    try {
      const result = await sendCommand(action, payload);
      setStatus(`${label}: ${result.message || "ok"}`);
      return result;
    } catch (err) {
      setStatus(`${label} failed: ${err.message}`);
      return null;
    }
  }

  // In-page dialog instead of window.confirm(), which would block the page.
  function confirmModal(text) {
    return new Promise((resolve) => {
      const modal = document.getElementById("modal");
      document.getElementById("modal-text").textContent = text;
      modal.hidden = false;
      const done = (answer) => {
        modal.hidden = true;
        resolve(answer);
      };
      document.getElementById("modal-ok").onclick = () => done(true);
      document.getElementById("modal-cancel").onclick = () => done(false);
    });
  }

  // ------------------------------------------------------------------- render

  function fmt(seconds) {
    return seconds === undefined ? "—" : `${seconds.toFixed(1)} s`;
  }

  function render(status) {
    state.status = status;
    const rec = status.recorder || {};
    const bridge = status.bridge || {};
    const recState = rec.online ? rec.state : "offline";
    const banner = recState.toLowerCase();
    document.getElementById("state").className = `state state-${banner}`;
    document.getElementById("rec-state").textContent = recState;
    let message = rec.online ? rec.message : "";
    if (rec.recording) message = `${rec.recording.frames} frames, ${fmt(rec.recording.duration_s)}`;
    document.getElementById("rec-message").textContent = message;

    let progress = 0;
    if (rec.playback) progress = rec.playback.t / Math.max(rec.playback.duration_s, 1e-3);
    else if (rec.segment) progress = rec.segment.t / Math.max(rec.segment.duration_s, 1e-3);
    document.getElementById("progress-bar").style.width = `${Math.min(100, 100 * progress)}%`;

    document.getElementById("bridge-state").textContent = bridge.online ? bridge.state.toUpperCase() : "offline";
    document.getElementById("motor-output").textContent =
      bridge.online ? (bridge.enable_lowcmd ? "ENABLED" : "disabled") : "—";
    document.getElementById("data-fresh").textContent = rec.online ? (rec.data_fresh ? "fresh" : "STALE") : "—";
    document.getElementById("joints").textContent = rec.online ? rec.joints : "—";
    document.getElementById("last-take").textContent = rec.last_saved || "—";
    document.getElementById("home").textContent = !rec.online ? "—"
      : rec.home ? `set ${rec.home.created.replace("T", " ")} (${rec.home.joints} joints)` : "not set";
    const offline = bridge.online ? (bridge.offline_joints || []) : [];
    document.getElementById("offline").textContent =
      !bridge.online ? "—" : offline.length ? offline.map((n) => n.replace(/_joint$/, "")).join(", ") : "none";

    renderRecordings(rec);
    renderGroupBoxes("record-groups", rec, rec.record_groups, null);
    renderGroupBoxes("groups", rec, rec.play_groups, selectedTakeGroups(rec));
    renderButtons(rec, bridge);
  }

  function renderRecordings(rec) {
    const list = rec.recordings || [];
    const key = JSON.stringify(list) + rec.selected_recording;
    if (key === state.recordingsKey) return;
    state.recordingsKey = key;
    const select = document.getElementById("recording");
    select.innerHTML = '<option value="">(none)</option>';
    for (const r of list) {
      const option = document.createElement("option");
      option.value = r.name;
      option.disabled = Boolean(r.error);
      option.textContent = r.error ? `${r.name} (unreadable)` : `${r.name} — ${fmt(r.duration_s)}, ${r.frames} frames`;
      select.appendChild(option);
    }
    select.value = rec.selected_recording || "";
  }

  // Groups contained in the selected recording (null: unknown / all).
  function selectedTakeGroups(rec) {
    const name = document.getElementById("recording").value;
    const take = (rec.recordings || []).find((r) => r.name === name);
    if (!take) return null;
    return take.groups || rec.groups || [];
  }

  // Checkboxes per joint group. `available` (or null for all) limits which can
  // be ticked; offline groups get a tag. Rebuilt only when inputs change, and
  // user ticks are kept across status updates.
  function renderGroupBoxes(id, rec, defaults, available) {
    const container = document.getElementById(id);
    const groups = rec.groups || [];
    const offline = rec.offline_groups || [];
    const key = JSON.stringify([groups, offline, available]);
    if (container.dataset.key === key) return;
    const previous = container.dataset.key
      ? Array.from(container.querySelectorAll("input:checked")).map((b) => b.value) : (defaults || groups);
    container.dataset.key = key;
    container.innerHTML = "";
    for (const g of groups) {
      const label = document.createElement("label");
      const box = document.createElement("input");
      box.type = "checkbox";
      box.value = g;
      const usable = !available || available.includes(g);
      box.disabled = !usable;
      box.checked = usable && previous.includes(g);
      label.classList.toggle("unavailable", !usable);
      label.appendChild(box);
      label.appendChild(document.createTextNode(` ${g.replace("_", " ")}`));
      if (offline.includes(g)) {
        const tag = document.createElement("span");
        tag.className = "tag";
        tag.textContent = "offline";
        label.appendChild(tag);
      }
      container.appendChild(label);
    }
  }

  function renderButtons(rec, bridge) {
    const s = rec.online ? rec.state : "offline";
    const record = document.getElementById("record");
    record.textContent = s === "RECORDING" ? "■ Stop recording" : "● Record";
    record.classList.toggle("recording", s === "RECORDING");
    record.disabled = !(s === "RECORDING" || s === "IDLE" || s === "HOLDING");
    document.getElementById("discard").disabled =
      !(s === "RECORDING" || (s === "IDLE" && rec.last_saved));
    document.getElementById("play").disabled =
      !((s === "IDLE" || s === "HOLDING") && document.getElementById("recording").value);
    const pause = document.getElementById("pause");
    pause.textContent = s === "PAUSED" ? "Resume" : "Stop";
    pause.disabled = !(s === "PLAYING" || s === "PAUSED");
    document.getElementById("reset").disabled = !rec.online || s === "RECORDING";
    document.getElementById("release").disabled =
      !["GOING_HOME", "APPROACHING", "PLAYING", "PAUSED", "RETURNING", "HOLDING"].includes(s);
    document.getElementById("set-home").disabled = !(s === "IDLE" || s === "HOLDING");
    document.getElementById("recording").disabled = !(s === "IDLE" || s === "HOLDING");
    const toggle = document.getElementById("motor-toggle");
    toggle.textContent = bridge.enable_lowcmd ? "Disable motor output" : "Enable motor output";
    toggle.disabled = !bridge.online;
  }

  // ------------------------------------------------------------------ actions

  function selectedGroups(id) {
    return Array.from(document.querySelectorAll(`#${id || "groups"} input:checked`)).map((b) => b.value);
  }


  document.getElementById("record").addEventListener("click", () => {
    const s = state.status && state.status.recorder && state.status.recorder.state;
    if (s === "RECORDING") {
      run("record_stop", null, "Stop recording");
    } else {
      const groups = selectedGroups("record-groups");
      if (!groups.length) {
        setStatus("select at least one limb to record");
        return;
      }
      run("record_start", {
        name: document.getElementById("rec-name").value.trim(),
        teach_damping_kd: parseFloat(document.getElementById("teach-kd").value) || 0,
        groups,
      }, "Record");
    }
  });

  document.getElementById("discard").addEventListener("click", async () => {
    const rec = state.status.recorder;
    const text = rec.state === "RECORDING" ? "Cancel the current recording without saving?"
      : `Delete the last take (${rec.last_saved})?`;
    if (await confirmModal(text)) run("discard", null, "Discard");
  });

  document.getElementById("teach-kd").addEventListener("change", (event) => {
    run("set_teach_damping", { value: parseFloat(event.target.value) || 0 }, "Teach damping");
  });

  document.getElementById("recording").addEventListener("change", (event) => {
    state.recordingsKey = ""; // re-render with the node's confirmed selection
    run("select", { file: event.target.value }, "Select");
  });

  document.getElementById("play").addEventListener("click", () => {
    const groups = selectedGroups();
    if (!groups.length) {
      setStatus("select at least one joint group");
      return;
    }
    run("play", { file: document.getElementById("recording").value, groups }, "Play");
  });

  document.getElementById("pause").addEventListener("click", () => {
    const paused = state.status.recorder.state === "PAUSED";
    run(paused ? "resume" : "pause", null, paused ? "Resume" : "Stop");
  });

  document.getElementById("reset").addEventListener("click", async () => {
    const file = document.getElementById("recording").value;
    const text = file
      ? `Reset: stop and move slowly ${state.status.recorder.home ? "home" : `to the start pose of ${file}`}?`
      : "Reset: stop the current motion? (no recording selected: the joints are released to damping)";
    if (await confirmModal(text)) {
      const groups = selectedGroups();
      run("reset", { file, groups: groups.length ? groups : null }, "Reset");
    }
  });

  document.getElementById("release").addEventListener("click", () => run("release", null, "Release"));

  document.getElementById("set-home").addEventListener("click", async () => {
    const text = state.status.recorder.home
      ? "Replace the home keyframe with the current joint positions?"
      : "Set the home keyframe to the current joint positions?";
    if (await confirmModal(text)) run("set_home", null, "Set home");
  });

  document.getElementById("motor-toggle").addEventListener("click", async () => {
    const enabled = state.status && state.status.bridge && state.status.bridge.enable_lowcmd;
    if (enabled) {
      run("set_enable_lowcmd", { enabled: false }, "Motor output");
      return;
    }
    const ok = await confirmModal(
      "Enable low-level motor output? The robot must be in DEBUG MODE and on a gantry. " +
      "Without playback commands the joints are only damped (teach mode).");
    if (ok) run("set_enable_lowcmd", { enabled: true }, "Motor output");
  });

  connect();
})();
