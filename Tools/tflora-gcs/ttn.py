from PyQt6.QtCore import QObject, pyqtSlot, pyqtSignal
import paho.mqtt.client as mqtt
import json
import base64

class TTNClient(QObject):
    up = pyqtSignal(str)

    def __init__(self,data):
        super(TTNClient, self).__init__()
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.username_pw_set(data["username"], data["password"])
        self.mqtt_client.connect(data["server"], 1883,60)
        self.mqtt_client.loop_start()

        self.app_id=data["application_id"]
        self.dev_id=data["device_id"]

    def on_connect(self, client, userdata, flags, rc):
        print("Connected with result code "+str(rc))
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.subscribe("v3/"+self.app_id+"@ttn/devices/"+self.dev_id+"/up")

    def on_message(self, client, userdata, message):
        try:
            message_payload = message.payload.decode("utf-8")
            payload_dict = json.loads(message_payload)
            print("recieved")

            if message.topic.endswith("up"):                
                dateTime = payload_dict["received_at"]
                balloon_id = payload_dict["end_device_ids"]["device_id"]
                latitude = payload_dict["uplink_message"]["decoded_payload"]["lat"]
                longitude = payload_dict["uplink_message"]["decoded_payload"]["lon"]
                altitude = payload_dict["uplink_message"]["decoded_payload"].get("alt_m")
                self.up.emit(f"Time:{dateTime}\nBalloon ID: {balloon_id}\nLatitude: {latitude}\nLongitude: {longitude}\nAltitude: {altitude}")

            if message.topic.endswith("sent"):
                print(message_payload)
                self.up.emit("Message sent")

            if message.topic.endswith("failed"):
                print(message_payload)
                self.up.emit("Message sent failed")

        except Exception as e:
                print("Exception:",e)

    def is_connected(self):
        return self.mqtt_client.is_connected()

    @pyqtSlot()
    def action(self):
        payload=bytearray(b'\x01\x02')
        p_b64=base64.b64encode(payload).decode('utf-8')
        data={
                "downlinks": [
                {
                "f_port": 15,
                "frm_payload": p_b64,
                "priority": "NORMAL"
                }
                ]
            }
        message = json.dumps(data)
        self.mqtt_client.publish("v3/"+self.app_id+"@ttn/devices/"+self.dev_id+"/down/replace", message)
        pass

    def disconnect_from_mqtt_broker(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()


