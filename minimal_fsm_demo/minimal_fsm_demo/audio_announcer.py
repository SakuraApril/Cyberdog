#!/usr/bin/env python3
"""
CyberDog 语音播报模块

根据官方QA:
- 仿真环境: gTTS/外部方案生成音频
- 真机: 调用语音SDK

Usage:
    from minimal_fsm_demo.audio_announcer import init_audio, say

    init_audio()
    say("检测到红色球体")
    say("即将通过独木桥")
    say("到达终点")
"""

import os
import sys
import tempfile
import threading

try:
    import lcm
except Exception:
    lcm = None

LCM_AUDIO_URL = "udpm://239.255.76.67:7671?ttl=255"
AUDIO_CMD_CHANNEL = "audio_cmd"

# --- 预设语音ID映射 ---
AUDIO_PRESETS = {
    "arrive": "到达终点",
    "detected_ball": "检测到球体",
    "start": "开始行走",
    "warning": "注意",
    "line_detected": "检测到赛道线",
    "curve_ahead": "前方弯道",
}


class AudioAnnouncer:
    """语音播报器.

    mode:
        "sim"   - 终端打印 (Docker仿真)
        "gtts"  - gTTS生成mp3 (需要网络)
        "host"  - Windows宿主机播放
        "lcm"   - 真机LCM发送
    """

    def __init__(self, mode="sim"):
        self.mode = mode
        self._lcm = None
        self._lock = threading.Lock()
        self._cache = {}
        self._cache_dir = tempfile.mkdtemp(prefix="cyberdog_tts_")

        if mode == "lcm" and lcm is not None:
            try:
                self._lcm = lcm.LCM(LCM_AUDIO_URL)
                threading.Thread(target=self._heartbeat, daemon=True).start()
                print("[Audio] LCM channel ready")
            except Exception as e:
                print(f"[Audio] LCM failed: {e}, fallback to sim")
                self.mode = "sim"

    def say(self, text):
        """播报文本."""
        if self.mode == "sim":
            print(f"[语音] {text}")
        elif self.mode == "gtts":
            self._say_gtts(text)
        elif self.mode == "host":
            self._say_host(text)
        elif self.mode == "lcm":
            self._say_lcm(text)

    def play_preset(self, key):
        """播放预设语音.

        Args:
            key: 预设名称，如 "arrive", "detected_ball"
        """
        text = AUDIO_PRESETS.get(key, key)
        self.say(text)

    def stop(self):
        """停止播放."""
        if self.mode == "host":
            try:
                import pygame
                pygame.mixer.music.stop()
            except Exception:
                pass

    def _say_gtts(self, text):
        """gTTS生成mp3."""
        if text in self._cache:
            path = self._cache[text]
        else:
            try:
                from gtts import gTTS
                path = os.path.join(self._cache_dir, f"{hash(text)}.mp3")
                tts = gTTS(text=text, lang='zh-cn')
                tts.save(path)
                self._cache[text] = path
            except Exception as e:
                print(f"[语音] {text} (TTS失败: {e})")
                return

        # 尝试用pygame播放
        try:
            import pygame
            if not pygame.mixer.get_init():
                pygame.mixer.init()
            pygame.mixer.music.load(path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                pygame.time.wait(100)
        except Exception:
            print(f"[语音] {text} (音频已保存: {path})")

    def _say_host(self, text):
        """Windows宿主机播放."""
        # 方法1: 用winsound + 预生成的wav
        # 方法2: 用 pyttsx3 (需要安装)
        try:
            import pyttsx3
            engine = pyttsx3.init()
            engine.say(text)
            engine.runAndWait()
        except ImportError:
            # fallback: 用os.system调用Windows SAPI
            # PowerShell TTS
            escaped = text.replace('"', '""')
            cmd = f'powershell -Command "Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak(\'{escaped}\')"'
            os.system(cmd)
        except Exception as e:
            print(f"[语音] {text} ({e})")

    def _say_lcm(self, text):
        """真机LCM发送."""
        import struct
        text_bytes = text.encode('utf-8')
        msg = struct.pack('i', 2) + struct.pack('I', len(text_bytes)) + text_bytes
        with self._lock:
            self._lcm.publish(AUDIO_CMD_CHANNEL, msg)

    def _heartbeat(self):
        while True:
            import time
            time.sleep(1.0)


# --- 全局接口 ---
_announcer = None


def init_audio(mode="sim"):
    """初始化语音播报.

    Args:
        mode: "sim"=仿真打印, "gtts"=生成音频, "host"=本机播放, "lcm"=真机
    """
    global _announcer
    _announcer = AudioAnnouncer(mode=mode)
    return _announcer


def say(text):
    """播报文本."""
    global _announcer
    if _announcer is None:
        _announcer = AudioAnnouncer(mode="sim")
    _announcer.say(text)


def play_preset(key):
    """播放预设语音."""
    global _announcer
    if _announcer is None:
        _announcer = AudioAnnouncer(mode="sim")
    _announcer.play_preset(key)
