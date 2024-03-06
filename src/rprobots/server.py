#!/usr/bin/env python3
"""Flask server for handling requests from the Wizard of Oz controller."""

import rospy
from rospy import Publisher
from std_msgs.msg import String

from flask import send_from_directory, Flask, request, jsonify

vector_speak_pub = Publisher("/behavior/say_text", String, queue_size=0)
misty_speak_pub = Publisher("/tts/speak", String, queue_size=0)
command_pub = Publisher("/role_playing_robots/cmd", String, queue_size=0)
name_pub = Publisher("/role_playing_robots/name", String, queue_size=0)

app = Flask(__name__)


@app.route("/vector-speech", methods=["POST"])
def handle_vector_message():

    vector_speak_pub.publish(request.json["text"])
    print("Vector saying ", request.json["text"])
    return jsonify(success=True)


@app.route("/misty-speech", methods=["POST"])
def handle_misty_message():
    misty_speak_pub.publish(request.json["text"])

    return jsonify(success=True)


@app.route("/scene-cmd", methods=["POST"])
def handle_scene_cmd():
    command_pub.publish(request.json["cmd"])

    return jsonify(success=True)


@app.route("/update-data", methods=["POST"])
def handle_update_data():
    if "name" in request.json:
        name_pub.publish("name_" + request.json["name"])
        name_pub.publish("condition_" + request.json["condition"])
    else:
        name_pub.publish(request.json["data"])

    return jsonify(success=True)


host_name = "0.0.0.0"
port = 5000

rospy.init_node("rprobots_server")
app.run(host=host_name, port=port, debug=True)
