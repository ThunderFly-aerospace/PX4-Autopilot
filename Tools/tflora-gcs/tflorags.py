import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QComboBox, QPushButton, QLabel, QWidget
import json
from PyQt6.QtWidgets import QDialog, QFormLayout, QLineEdit, QMessageBox

from ttn import TTNClient   


class MyApp(QMainWindow):
    def __init__(self,data):
        super().__init__()

        self.tnn = TTNClient(data)
        self.tnn.up.connect(self.update_status)

        self.setWindowTitle("TFLORA TNN GS")
        self.setGeometry(100, 100, 400, 200)

        layout = QVBoxLayout()

        self.button = QPushButton("Fire Action")
        self.button.clicked.connect(self.fire_action)
        layout.addWidget(self.button)


        self.status_label = QLabel("Status: ")
        layout.addWidget(self.status_label)

        self.queue_label = QLabel("Action Queue: ")
        layout.addWidget(self.queue_label)

        widget = QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

    def update_status(self, status):
        self.status_label.setText(status)

    def fire_action(self):
        self.tnn.action()

    def clear_queue(self):
        self.queue_label.setText("Action Queue: ")

if __name__ == "__main__":
    app = QApplication(sys.argv)

    #load config.json
    ok = True
    data={}
    try:
        with open('config.json') as f:
            data = json.load(f)
            #check loaded data
            if "server" not in data or "username" not in data or "password" not in data or "application_id" not in data or "device_id" not in data:            
                raise Exception("failed load config.json: missing key")
            
            if data["server"]=="" or data["username"]=="" or data["password"]=="" or data["application_id"]=="" or data["device_id"]=="":
                raise Exception("failed load config.json: empty value")

    except Exception as e:
        print(e)
        print("Creating empty config.json")
        data = {}
        data['server'] = ""
        data['username'] = ""
        data['password'] = ""
        data['application_id'] = ""
        data['device_id'] = ""
        with open('config.json', 'w') as f:
            json.dump(data, f, indent=4)
        ok = False  

    if not ok:
        QMessageBox.warning(None, "Warning", "Please fill the config.json file", QMessageBox.StandardButton.Ok)
        sys.exit(0)

    window = MyApp(data)
    window.show()
    sys.exit(app.exec())