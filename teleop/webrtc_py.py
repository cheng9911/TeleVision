# webrtc_vuer_demo.py
import cv2
import numpy as np
from vuer import Vuer
from vuer.schemas import WebRTCStereoVideoPlane, Hands, DefaultScene
from aiohttp import web
import logging
import ssl
import os
import asyncio


# ================= WebRTC 服务端 =================
class WebRTCServer:
    def __init__(self, port=8080):
        self.port = port
        self.app = web.Application()
        self.setup_routes()

    def setup_routes(self):
        self.app.router.add_get("/", self.index)
        self.app.router.add_get("/client.js", self.javascript)
        self.app.router.add_post("/offer", self.offer)

    async def index(self, request):
        return web.Response(
            content_type="text/html",
            text="""
            <html>
                <body>
                    <h1>WebRTC Stream Server</h1>
                </body>
            </html>
            """
        )

    async def javascript(self, request):
        return web.Response(
            content_type="application/javascript",
            text=self.get_client_js()
        )

    async def offer(self, request):
        # 此处应实现WebRTC信令交换逻辑
        return web.Response(
            content_type="application/json",
            text='{"status": "ready"}'
        )

    def get_client_js(self):
        # 返回WebRTC客户端JS代码
        return """
        // WebRTC客户端逻辑
        console.log("WebRTC Client Loaded");
        """

    def run(self, cert_file, key_file):
        ssl_context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_context.load_cert_chain(cert_file, key_file)
        web.run_app(
            self.app,
            port=self.port,
            ssl_context=ssl_context,
            access_log=None
        )


# ================= Vuer 客户端 =================
class WebRTCVisualizer:
    def __init__(self):
        self.app = Vuer(host="0.0.0.0", port=8012, cert_file="cert.pem", key_file="key.pem")
        self.app.add_handler("HANDS_MOVE")(self.on_hand_move)

    async def on_hand_move(self, event, session):
        # 处理手势数据
        print("Received hand data:", event.value)

    async def main(self, session):
        session.set @ DefaultScene()

        # 添加WebRTC视频平面
        session.upsert @ WebRTCStereoVideoPlane(
            src="https://localhost:8080/offer",
            key="webrtc-stream",
            position=[0, 1.5, -1],
            scale=2,
            aspectRatio=16 / 9,
            autoPlay=True,
            muted=True
        )

        # 添加手部追踪组件
        session.upsert @ Hands(fps=30, stream=True, key="hands")

        while True:
            await session.update()
            await asyncio.sleep(1 / 30)

    def run(self):
        self.app.run(self.main)


# ================= 主程序 =================
if __name__ == "__main__":
    # 生成SSL证书（首次运行需要）
    if not os.path.exists("cert.pem") or not os.path.exists("key.pem"):
        os.system(
            "openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes -subj '/CN=localhost'")

    # 启动WebRTC服务端
    server = WebRTCServer(port=8080)
    from multiprocessing import Process

    server_process = Process(target=server.run, args=("cert.pem", "key.pem"))
    server_process.start()

    # 启动Vuer客户端
    visualizer = WebRTCVisualizer()
    visualizer.run()