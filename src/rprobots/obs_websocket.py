"""Helper module to interact with OBS Studio using WebSockets."""
from obswebsocket import obsws, requests


class Websocket:
    def __init__(self, ip, port, password) -> None:
        self.ws = obsws(ip, port, password)
        self.ws.connect()

    def set_text(self, source: str, text: str):
        """Set the text of a Text Freetype 2 source."""
        self.ws.call(requests.SetTextFreetype2Properties(source=source, text=text))

    def play_audio(self, source: str, file: str):
        """Play an audio file in a media source, given the name of a local file."""
        self.ws.call(
            requests.SetSourceSettings(
                sourceName=source, sourceSettings={"local_file": file}
            )
        )

    def switch_scene(self, scene_name: str):
        """Switch to a different scene."""
        self.ws.call(requests.SetCurrentScene(scene_name=scene_name))
        requests.SetSceneItemRender("Browser", False)

    def enable_scene_item(self, item_name: str, enable: bool):
        """Enable or disable a scene item."""
        self.ws.call(requests.SetSceneItemRender(item_name, enable))
