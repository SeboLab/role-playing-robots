from PyQt5.QtWidgets import QMainWindow, QStackedWidget, QApplication
import requests
import qtwizard.rpwizard_ui as rpwizard_ui
import sys
import json
import os
from time import sleep

SERVER_IP = "100.118.30.93"
SERVER = f"http://{SERVER_IP}:5000"

CONDITIONS = ["Control", "Narrative", "Game"]
DEBUG_GUI = False


def send_command(endpoint, data):
    print(f"{SERVER}/{endpoint}", data)
    if not DEBUG_GUI:
        requests.post(f"{SERVER}/{endpoint}", json=data)


class MainWindow(QMainWindow, rpwizard_ui.Ui_mainWindow):
    def __init__(self):
        super(self.__class__, self).__init__()
        self.setupUi(self)
        self.widget = QStackedWidget()
        self.widget.addWidget(self)
        self.widget.show()

        self.index = 0
        self.commandsA = {
            "Control": ["a2_c", "a3_c", "a4_c"],
            "Narrative": [
                "a1_n2",
                "a2_n1",
                "a2_n2",
                "a3_n1",
                ["a3_n1a", "a3_n1b", "a3_n1c"],
                "a4_c",
            ],
            "Game": ["a2_c", "a3_c", "a4_g1", "a4_g2", "a4_g3"],
        }

        self.commandsB = {
            "Control": ["b1_c", "b2_c", "b3_c", "b4_c"],
            "Narrative": [
                "b1_n1",
                ["b1_n1a", "b1_n1b", "b1_n1c"],
                "b1_n2",
                "b1_n3",
                "b2_n1",
                ["b2_n1a", "b2_n1b", "b2_n1c"],
                "b2_n2",
                "b3_c",
                "b4_n1",
                "b4_n2",
            ],
            "Game": [
                "b1_c",
                "b2_c",
                "b3_g1",
                "b3_g2",
                "b3_g3",
                "b3_g4",
                "b3_g5",
                "b3_g6",
                "b3_g7",
                "b3_g8",
                "b3_g9",
                "b3_g10",
                "b4_c",
            ],
        }

        self.commandsC = {
            "Control": ["c0", "c1", "c2", "c3", "c5", "c55", ["c5a", "c5b"], "c6"],
            "Narrative": ["c0", "c1", "c2", "c3", "c5", "c55", ["c5a", "c5b"], "c6"],
            "Game": ["c0", "c1", "c2", "c3", "c5", "c55", ["c5a", "c5b"], "c6"],
        }

        self.trial_data = {}
        self.commands = [self.commandsA, self.commandsB, self.commandsC]
        self.cond = "Control"
        self.vectorBtn.clicked.connect(self.vector_speak)
        self.mistyBtn.clicked.connect(self.misty_speak)
        self.startBtn.clicked.connect(self.start_reset)
        self.cmdBtn.clicked.connect(self.send_sequence_cmd)

        self.goBtnA.clicked.connect(lambda _: self.go_click("yes", self.commandsA))
        self.incorrectBtnA.clicked.connect(
            lambda _: self.go_click("no", self.commandsA)
        )

        self.goBtnB.clicked.connect(lambda _: self.go_click("yes", self.commandsB))
        self.incorrectBtnB.clicked.connect(
            lambda _: self.go_click("no", self.commandsB)
        )

        self.goBtnC.clicked.connect(lambda _: self.go_click("yes", self.commandsC))
        self.incorrectBtnC.clicked.connect(
            lambda _: self.go_click("no", self.commandsC)
        )
        self.invalidBtnC.clicked.connect(
            lambda _: self.go_click("invalid", self.commandsC)
        )

        self.finishBtn.clicked.connect(self.export_data)

    def go_click(self, data, commands):
        send_command("update-data", {"data": f"puzzle_{data}"})
        self.advance_choice(commands)

    def advance_choice(self, commands):
        button_list = commands[self.cond]
        print(button_list)
        for i, button_name in enumerate(button_list):
            if type(button_name) == list:
                for name in button_name:
                    button = getattr(self, name)
                    if button.isChecked():
                        button.setChecked(False)
                        send_command(
                            "update-data", {"data": f"location_{button.text()}"}
                        )
                        sleep(0.1)
                        send_command(
                            "scene-cmd", {"cmd": f"{button_name[0][:2]}_choices"}
                        )
                        if i + 1 != len(button_list):
                            getattr(self, button_list[i + 1]).setChecked(True)
                        else:
                            self.advance_scenario()
                        return
            elif getattr(self, button_name).isChecked():
                send_command("scene-cmd", {"cmd": button_name})
                if i + 1 != len(button_list) and button_name not in ("c2", "c3"):
                    if type(button_list[i + 1]) == list:
                        getattr(self, button_list[i + 1][0]).setChecked(True)
                    else:
                        getattr(self, button_list[i + 1]).setChecked(True)
                else:
                    self.advance_scenario()
                return

    def advance_scenario(self):
        if self.scenarios.currentIndex() + 1 < 4:
            getattr(
                self, self.commands[self.scenarios.currentIndex()][self.cond][0]
            ).setChecked(True)
            self.scenarios.setCurrentIndex(self.scenarios.currentIndex() + 1)

    def start_reset(self):
        for i in range(1, 4):
            if getattr(self, f"cond{i}").isChecked():
                self.cond = CONDITIONS[i - 1]
                break

        self.trial_data = {
            "name": self.participantName.toPlainText(),
            "id": int(self.participantId.toPlainText()),
            "condition": self.cond,
        }

        send_command("scene-cmd", {"cmd": "reset"})
        send_command("update-data", self.trial_data)
        self.advance_scenario()

    def vector_speak(self):
        data = {"text": str(self.vectorField.toPlainText())}
        send_command("vector-speech", data)

    def misty_speak(self):
        data = {"text": str(self.mistyField.toPlainText())}
        send_command("misty-speech", data)

    def send_sequence_cmd(self):
        data = {"cmd": self.commands[self.index % len(self.commands)]}
        send_command("scene-cmd", data)
        self.index += 1

    def export_data(self):
        os.makedirs("data/", exist_ok=True)
        with open(f"data/P{self.trial_data['id']: 03d}.json", "w+") as f:
            json.dump(self.trial_data, f)
        print(f"data/P{self.trial_data['id']: 03d}.json exported")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    wizard = MainWindow()
    wizard.show()
    sys.exit(app.exec_())
