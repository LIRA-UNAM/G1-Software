// Timers in a dedicated worker keep running at full rate when the tab is in
// the background (main-thread timers are throttled to ~1 Hz there), so the
// dead-man heartbeat does not stop just because another tab is focused.
setInterval(() => postMessage("tick"), 100);
