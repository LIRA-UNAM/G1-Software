import asyncio
from concurrent.futures import ThreadPoolExecutor
import json
import signal
import threading

from aiohttp import WSMsgType, web
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .bridge import GetupGuiBridge


STATIC_ASSETS = {"app.js", "styles.css", "heartbeat_worker.js"}
STATUS_PERIOD_SEC = 0.1


class GetupGuiServer:
    """aiohttp + WebSocket server for a GUI bridge node.

    Also used by other GUIs (e.g. motion_recorder_gui): the bridge must provide
    host/port, config(), heartbeat(), publish_estop(), status_snapshot() and
    execute_command(command) (handling "estop_services"); static files are
    served from <package_name>'s share directory.
    """

    def __init__(self, bridge, package_name="getup_gui"):
        self.bridge = bridge
        self.clients = set()
        # Blocking ROS calls run here; several workers so an e-stop
        # confirmation never waits behind a slow parameter read.
        self.workers = ThreadPoolExecutor(max_workers=4)
        self.app = web.Application(client_max_size=1024 * 1024)
        self.static_dir = get_package_share_directory(package_name) + "/static"
        self.app.add_routes(
            [
                web.get("/", self.index),
                web.get("/health", self.health),
                web.get("/ws", self.websocket),
                web.get("/static/{asset}", self.static_asset),
            ]
        )

    async def index(self, _request):
        return web.FileResponse(
            self.static_dir + "/index.html",
            headers={"Cache-Control": "no-store"},
        )

    async def static_asset(self, request):
        asset = request.match_info["asset"]
        if asset not in STATIC_ASSETS:
            raise web.HTTPNotFound()
        return web.FileResponse(
            self.static_dir + "/" + asset,
            headers={"Cache-Control": "no-cache"},
        )

    async def health(self, _request):
        return web.json_response({"ok": True, "clients": len(self.clients)})

    async def websocket(self, request):
        ws = web.WebSocketResponse(heartbeat=5.0, receive_timeout=30.0)
        await ws.prepare(request)
        self.clients.add(ws)
        await ws.send_json({"type": "hello", "config": self.bridge.config()})

        try:
            async for message in ws:
                if message.type == WSMsgType.TEXT:
                    await self._handle_message(ws, message.data)
                elif message.type == WSMsgType.ERROR:
                    self.bridge.get_logger().warning(
                        "websocket error: {}".format(ws.exception())
                    )
        finally:
            self.clients.discard(ws)
        return ws

    async def _handle_message(self, ws, raw_message):
        request_id = None
        try:
            command = json.loads(raw_message)
            if not isinstance(command, dict):
                raise ValueError("command_must_be_object")
            action = command.get("action")
            # Fast paths: handled inline on the event loop, never queued.
            if action == "heartbeat":
                self.bridge.heartbeat()
                return
            request_id = command.get("request_id")
            if action == "estop":
                self.bridge.publish_estop()
                command = {"action": "estop_services"}
            loop = asyncio.get_event_loop()
            # run_in_executor instead of asyncio.to_thread: Foxy ships Python 3.8.
            result = await loop.run_in_executor(
                self.workers, self.bridge.execute_command, command)
            result.update({"type": "ack", "request_id": request_id})
        except (ValueError, RuntimeError, json.JSONDecodeError) as exc:
            result = {"type": "ack", "request_id": request_id, "ok": False, "error": str(exc)}
        if not ws.closed:
            await ws.send_json(result)

    async def status_loop(self):
        while True:
            await asyncio.sleep(STATUS_PERIOD_SEC)
            if not self.clients:
                continue
            payload = {"type": "status", "status": self.bridge.status_snapshot()}
            for ws in list(self.clients):
                try:
                    await ws.send_json(payload)
                except (ConnectionResetError, RuntimeError):
                    self.clients.discard(ws)


def run_server(bridge_cls, package_name, args=None):
    """Spins bridge_cls() on a ROS thread and serves its GUI until SIGINT/SIGTERM."""
    try:
        # Let the asyncio loop below own SIGINT/SIGTERM so shutdown happens in
        # order (rclpy's own handler would kill the ROS thread first).
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:  # Foxy: no signal handler options
        rclpy.init(args=args)
    bridge = bridge_cls()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(bridge)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    server = GetupGuiServer(bridge, package_name)
    runner = web.AppRunner(server.app, access_log=None)

    async def run():
        await runner.setup()
        site = web.TCPSite(runner, bridge.host, bridge.port)
        await site.start()
        bridge.get_logger().info(
            "{} available at http://{}:{}".format(package_name, bridge.host, bridge.port)
        )
        status_task = asyncio.ensure_future(server.status_loop())
        stop_event = asyncio.Event()
        loop = asyncio.get_event_loop()
        for signame in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(signame, stop_event.set)
            except NotImplementedError:
                pass
        await stop_event.wait()
        status_task.cancel()
        await runner.cleanup()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass
    finally:
        server.workers.shutdown(wait=False)
        executor.shutdown()
        bridge.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=2.0)


def main(args=None):
    run_server(GetupGuiBridge, "getup_gui", args)


if __name__ == "__main__":
    main()
