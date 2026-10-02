(function () {
  const APPLY_DEBOUNCE_MS = 400;

  const state = {
    ws: null,
    requestId: 0,
    pending: new Map(),
    config: null,
    values: { policy: {}, bridge: {} }, // last values read from / applied to the nodes
    dirty: {}, // "node.name" -> true while an edit is waiting to be applied
    timers: {},
    status: null,
    worker: null,
  };

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
      for (const { reject } of state.pending.values()) {
        reject(new Error("disconnected"));
      }
      state.pending.clear();
      setTimeout(connect, 1000);
    };
    state.ws.onerror = () => setStatus("connection error");
    state.ws.onmessage = (event) => handleMessage(JSON.parse(event.data));
  }

  function startHeartbeat() {
    const send = () => {
      if (state.ws && state.ws.readyState === WebSocket.OPEN) {
        state.ws.send('{"action":"heartbeat"}');
      }
    };
    try {
      state.worker = new Worker("/static/heartbeat_worker.js");
      state.worker.onmessage = send;
    } catch (err) {
      setInterval(send, 100); // fallback: throttled in background tabs
    }
  }

  function setStatus(text) {
    document.getElementById("status").textContent = text;
  }

  function handleMessage(message) {
    if (message.type === "hello") {
      state.config = message.config;
      document.getElementById("policy-node").textContent = message.config.policy_node;
      document.getElementById("bridge-node").textContent = message.config.bridge_node;
      buildScalarForm("policy", message.config.policy_specs);
      buildScalarForm("bridge", message.config.bridge_specs);
      readParameters();
      return;
    }
    if (message.type === "status") {
      renderStatus(message.status);
      return;
    }
    if (message.type === "ack" && state.pending.has(message.request_id)) {
      const { resolve, reject } = state.pending.get(message.request_id);
      state.pending.delete(message.request_id);
      if (message.ok) {
        resolve(message);
      } else {
        reject(new Error(message.error || message.reason || message.message || "request_failed"));
      }
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

  // ------------------------------------------------------------------- e-stop

  async function estop() {
    const button = document.getElementById("estop");
    button.classList.add("pressed");
    setTimeout(() => button.classList.remove("pressed"), 300);
    try {
      const result = await sendCommand("estop");
      const r = result.results || {};
      setStatus(`E-STOP sent (bridge ${r.bridge_estop ? "latched" : "?"}, policy ${r.policy_stop ? "stopped" : "?"})`);
    } catch (err) {
      setStatus(`E-STOP: ${err.message}`);
    }
  }

  function onKey(event) {
    if (event.key !== "Escape" && event.key !== " ") {
      return;
    }
    const el = document.activeElement;
    const typing = el && el.tagName === "INPUT" && el.type === "text";
    if (event.key === " " && typing) {
      return;
    }
    event.preventDefault();
    estop();
  }

  // --------------------------------------------------- confirmation (in page)

  // Never window.confirm(): a native dialog pauses page timers and the
  // websocket handlers, which would starve the heartbeat and trip the dead-man.
  function confirmModal(text) {
    return new Promise((resolve) => {
      const modal = document.getElementById("modal");
      document.getElementById("modal-text").textContent = text;
      modal.hidden = false;
      const done = (answer) => {
        modal.hidden = true;
        document.getElementById("modal-ok").onclick = null;
        document.getElementById("modal-cancel").onclick = null;
        resolve(answer);
      };
      document.getElementById("modal-ok").onclick = () => done(true);
      document.getElementById("modal-cancel").onclick = () => done(false);
    });
  }

  // -------------------------------------------------------------- status view

  function fmtAge(age) {
    if (age === undefined || age < 0) return "never";
    return age < 1 ? `${(age * 1000).toFixed(0)} ms` : `${age.toFixed(1)} s`;
  }

  function renderStatus(status) {
    state.status = status;
    const bridge = status.bridge || {};
    const policy = status.policy || {};
    const section = document.getElementById("state");
    const bridgeState = bridge.online ? bridge.state : "offline";
    section.className = `state state-${bridgeState}`;
    document.getElementById("bridge-state").textContent = bridgeState.toUpperCase();
    document.getElementById("estop-reason").textContent =
      bridge.online && bridge.estop ? `(${bridge.estop_reason})` : "";
    document.getElementById("policy-mode").textContent = policy.online ? policy.mode : "offline";
    document.getElementById("policy-step").textContent =
      policy.online ? `${policy.step} (settle ${policy.settle_steps})` : "—";
    document.getElementById("motor-output").textContent =
      bridge.online ? (bridge.enable_lowcmd ? "ENABLED" : "disabled") : "—";
    document.getElementById("deadman").textContent = !bridge.online ? "—"
      : bridge.heartbeat_timeout_s <= 0 ? "off"
      : bridge.deadman_armed ? `armed (${bridge.heartbeat_timeout_s.toFixed(2)} s)` : "waiting for heartbeat";
    document.getElementById("data-fresh").textContent =
      policy.online ? (policy.data_fresh ? "fresh" : "STALE") : "—";
    document.getElementById("lowstate-age").textContent = bridge.online ? fmtAge(bridge.lowstate_age) : "—";
    document.getElementById("inference").textContent =
      policy.online ? `${policy.inference_ms.toFixed(2)} ms` : "—";
    document.getElementById("model-path").textContent = policy.online ? policy.policy_path : "—";

    const toggle = document.getElementById("motor-toggle");
    toggle.textContent = bridge.enable_lowcmd ? "Disable motor output" : "Enable motor output";
    toggle.disabled = !bridge.online;
    document.getElementById("reset-estop").disabled = !(bridge.online && bridge.estop);
    document.getElementById("policy-start").disabled =
      !policy.online || policy.mode !== "IDLE" || (bridge.online && bridge.estop);
    document.getElementById("policy-stop").disabled = !policy.online || policy.mode === "IDLE";
    document.getElementById("policy-reload").disabled = !policy.online || policy.mode !== "IDLE";
  }

  // ---------------------------------------------------------- parameter forms

  function key(node, name) {
    return `${node}.${name}`;
  }

  function buildScalarForm(node, specs) {
    const container = document.getElementById(`params-${node}`);
    container.innerHTML = "";
    for (const spec of specs) {
      const row = document.createElement("div");
      row.className = "param-row";
      const label = document.createElement("label");
      const text = spec.unit ? `${spec.label} (${spec.unit})` : spec.label;
      label.textContent = text;
      label.title = `${spec.name}: ${text}`;
      label.setAttribute("for", `param_${node}_${spec.name}`);
      const input = document.createElement("input");
      input.id = `param_${node}_${spec.name}`;
      if (spec.type === "bool") {
        input.type = "checkbox";
        input.addEventListener("change", () => apply(node, spec.name, input.checked));
      } else {
        input.type = "number";
        input.step = spec.step;
        input.min = spec.min;
        input.max = spec.max;
        input.addEventListener("input", () => schedule(node, spec.name, () => {
          if (input.value === "") return undefined;
          return spec.type === "int" ? parseInt(input.value, 10) : parseFloat(input.value);
        }));
      }
      row.appendChild(label);
      row.appendChild(input);
      container.appendChild(row);
    }
  }

  function buildObservationScales(terms, scales) {
    const container = document.getElementById("obs-scales");
    container.innerHTML = "";
    (terms || []).forEach((term, i) => {
      const row = document.createElement("div");
      row.className = "param-row";
      const label = document.createElement("label");
      label.textContent = term;
      const input = document.createElement("input");
      input.type = "number";
      input.step = 0.01;
      input.dataset.arrayIndex = i;
      input.value = scales && scales.length === terms.length ? scales[i] : 1.0;
      input.addEventListener("input", () => editArrayElement("policy", "observation_scales", i, input));
      row.appendChild(label);
      row.appendChild(input);
      container.appendChild(row);
    });
  }

  function buildJointTable() {
    const policy = state.values.policy;
    const bridge = state.values.bridge;
    const names = policy.joint_names || bridge.joint_names || [];
    const warning = document.getElementById("joint-warning");
    const mismatch = policy.joint_names && bridge.joint_names &&
      JSON.stringify(policy.joint_names) !== JSON.stringify(bridge.joint_names);
    warning.hidden = !mismatch;
    warning.textContent = mismatch ? "Policy and bridge joint_names differ: bridge columns are matched by name." : "";

    const table = document.getElementById("joint-table");
    table.innerHTML = "";
    const head = table.createTHead().insertRow();
    head.insertCell().textContent = "#";
    head.insertCell().textContent = "Joint";
    for (const col of state.config.joint_columns) {
      const th = head.insertCell();
      th.textContent = col.unit ? `${col.label} (${col.unit})` : col.label;
      th.className = `col-${col.node}`;
    }
    const body = table.createTBody();
    names.forEach((joint, row) => {
      const tr = body.insertRow();
      tr.insertCell().textContent = row;
      tr.insertCell().textContent = joint.replace(/_joint$/, "");
      for (const col of state.config.joint_columns) {
        const td = tr.insertCell();
        td.className = `col-${col.node}`;
        const values = state.values[col.node][col.name];
        // Bridge arrays are indexed in the bridge's own joint order.
        const nodeNames = state.values[col.node].joint_names || names;
        const index = nodeNames.indexOf(joint);
        const input = document.createElement("input");
        input.type = "number";
        input.step = col.step;
        input.id = `joint_${col.node}_${col.name}_${index}`;
        if (!Array.isArray(values) || index < 0) {
          input.disabled = true;
        } else {
          // action_scale may be a single value for all joints.
          input.value = values.length === 1 ? values[0] : values[index];
          input.addEventListener("input", () => editArrayElement(col.node, col.name, index, input));
        }
        td.appendChild(input);
      }
    });
  }

  function editArrayElement(node, name, index, input) {
    schedule(node, name, () => {
      if (input.value === "") return undefined;
      let values = (state.values[node][name] || []).slice();
      const count = (state.values[node].joint_names || []).length;
      if (name === "action_scale" && values.length === 1 && count > 1) {
        values = new Array(count).fill(values[0]); // expand to per-joint on first edit
      }
      values[index] = parseFloat(input.value);
      return values;
    });
  }

  // Debounced auto-apply as in walk_gui: edits take effect shortly after the
  // last keystroke, without an Apply button.
  function schedule(node, name, getValue) {
    const k = key(node, name);
    state.dirty[k] = true;
    clearTimeout(state.timers[k]);
    state.timers[k] = setTimeout(() => {
      delete state.timers[k];
      const value = getValue();
      if (value !== undefined) apply(node, name, value);
    }, APPLY_DEBOUNCE_MS);
  }

  async function apply(node, name, value) {
    const k = key(node, name);
    try {
      const result = await sendCommand("set_parameters", { node, parameters: { [name]: value } });
      state.values[node][name] = value;
      setStatus(`${node}: ${name} applied`);
      markInput(node, name, "applied");
      return result;
    } catch (err) {
      setStatus(`${node}: ${name} REJECTED (${err.message})`);
      markInput(node, name, "rejected");
      delete state.dirty[k];
      await readParameters(); // show what the node actually uses
      return null;
    } finally {
      delete state.dirty[k];
    }
  }

  function markInput(node, name, cls) {
    const inputs = document.querySelectorAll(
      `[id^="param_${node}_${name}"], [id^="joint_${node}_${name}_"]`);
    for (const input of inputs) {
      input.classList.remove("applied", "rejected");
      input.classList.add(cls);
      setTimeout(() => input.classList.remove(cls), 1200);
    }
  }

  function showValues() {
    for (const node of ["policy", "bridge"]) {
      const specs = state.config[`${node}_specs`];
      for (const spec of specs) {
        if (state.dirty[key(node, spec.name)]) continue;
        const input = document.getElementById(`param_${node}_${spec.name}`);
        const value = state.values[node][spec.name];
        if (!input || value === undefined || value === null) continue;
        if (spec.type === "bool") input.checked = value;
        else input.value = value;
      }
    }
    const policy = state.values.policy;
    buildObservationScales(policy.observation_terms, policy.observation_scales);
    const pathInput = document.getElementById("policy-path");
    if (document.activeElement !== pathInput && policy.policy_path !== undefined) {
      pathInput.value = policy.policy_path;
    }
    buildJointTable();
  }

  async function readParameters() {
    try {
      const result = await sendCommand("read_parameters");
      for (const node of ["policy", "bridge"]) {
        state.values[node] = result.parameters[node] || {};
      }
      showValues();
      const errors = Object.entries(result.errors || {}).map(([n, e]) => `${n}: ${e}`);
      setStatus(errors.length ? `read: ${errors.join(", ")}` : "parameters loaded");
    } catch (err) {
      setStatus(`read error: ${err.message}`);
    }
  }

  // ----------------------------------------------------------------- actions

  async function simple(action, label) {
    try {
      const result = await sendCommand(action);
      setStatus(`${label}: ${result.message || "ok"}`);
    } catch (err) {
      setStatus(`${label} failed: ${err.message}`);
    }
  }

  async function resetEstop() {
    if (await confirmModal("Reset the e-stop? The robot stays in damping until the policy is started again.")) {
      simple("reset_estop", "Reset e-stop");
    }
  }

  async function toggleMotorOutput() {
    const enabled = state.status && state.status.bridge && state.status.bridge.enable_lowcmd;
    if (enabled) {
      apply("bridge", "enable_lowcmd", false);
      return;
    }
    const ok = await confirmModal(
      "Enable low-level motor output? The robot must be in DEBUG MODE (built-in controller released) " +
      "and on a gantry. Without commands the motors go to damping.");
    if (ok) apply("bridge", "enable_lowcmd", true);
  }

  async function reloadPolicy() {
    const path = document.getElementById("policy-path").value.trim();
    if (!path) return;
    if (await confirmModal(`Load policy ${path}?`)) {
      const result = await apply("policy", "policy_path", path);
      if (result) setStatus(`policy loaded: ${path}`);
    }
  }

  document.getElementById("estop").addEventListener("click", estop);
  document.addEventListener("keydown", onKey, true);
  document.getElementById("reset-estop").addEventListener("click", resetEstop);
  document.getElementById("policy-start").addEventListener("click", () => simple("policy_start", "Start policy"));
  document.getElementById("policy-stop").addEventListener("click", () => simple("policy_stop", "Stop policy"));
  document.getElementById("motor-toggle").addEventListener("click", toggleMotorOutput);
  document.getElementById("policy-reload").addEventListener("click", reloadPolicy);
  document.querySelectorAll("button.refresh").forEach((b) => b.addEventListener("click", readParameters));

  startHeartbeat();
  connect();
})();
