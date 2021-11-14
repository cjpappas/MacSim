from flask import Flask, redirect, send_from_directory, url_for

app = Flask(__name__, static_url_path="")

@app.route("/")
def index():
    return redirect(url_for("hud"))

@app.route("/hud")
def hud():
    return send_from_directory("", "hud.html")

@app.route("/api")
def api():
    return send_from_directory("", "api.js")