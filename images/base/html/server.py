from flask import Flask, redirect, request, send_from_directory, url_for
import subprocess
import time

app = Flask(__name__, static_url_path="")
sims = { "station_keeping": "station_keeping.launch" }

@app.route("/")
def index():
    return redirect(url_for("hud"))

@app.route("/hud")
def hud():
    return send_from_directory("", "hud.html")

@app.route("/api.js")
def api():
    return send_from_directory("", "api.js")

@app.route("/api/start_sim", methods=["POST"])
def start():
    sim = request.json["sim"]
    if sim in sims.keys():
        subprocess.run(["source ~/vrx_ws/devel/setup.bash && roslaunch vrx_gazebo %s gui:=false"%(sims[sim])],
                        executable="/bin/bash", shell=True)
        return {"status": "Simulation started."}, 200
    return {"status": "Simulation type not found"}, 404
    