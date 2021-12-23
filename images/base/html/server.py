from flask import Flask, request, send_from_directory
import subprocess

app = Flask(__name__, static_url_path="")
sims = { 
    "station_keeping": "station_keeping.launch",
    "wayfinding": "wayfinding.launch",
    "perception": "perception_task.launch" ,
    "wildlife": "wildlife.launch",
    "gymkhana": "gymkhana.launch",
    "scan_dock_deliver": "scan_dock_deliver.launch"
}

@app.route("/")
def index():
    return send_from_directory("", "landing.html") 

@app.route("/hud")
def hud():
    return send_from_directory("", "hud.html")

@app.route("/style")
def style():
    return send_from_directory("", "style.css")

@app.route("/api.js")
def api():
    return send_from_directory("", "api.js")

@app.route("/get_image")
def get_image():
    return send_from_directory("", "no-camera-found.png")

@app.route("/api/start_sim", methods=["POST"])
def start():
    sim = request.json["sim"]
    if sim in sims.keys():
        # Need a virtual display to force gazebo to add Camera sensors and publish
        # https://github.com/PX4/PX4-containers/issues/198
        subprocess.run(["source ~/vrx_ws/devel/setup.bash && (Xvfb :1 -screen 0 1600x1200x16 &) && export DISPLAY=:1.0 && roslaunch vrx_gazebo %s gui:=false &"%(sims[sim])],
                                 executable="/bin/bash", shell=True)
        return {"status": "Simulation started."}
    return {"status": "Simulation type not found."}

@app.route("/api/stop_sim", methods=["POST"])
def stop():
    subprocess.run(["source ~/vrx_ws/devel/setup.bash && rosnode kill gazebo && killall -9 gzserver"],
                    executable="/bin/bash", shell=True)
    return {}, 200
