var map;
var marker;
var goal;
var sinks = {};

function initMap() {
    map = new google.maps.Map(document.getElementById('map'), {
        center: {lat: -33.722, lng: 150.67398},
        zoom: 18,
        mapTypeId: "satellite",
    });
    marker = new google.maps.Marker({
        position: {lat: -33.722, lng:150.67398},
        map,
        title: "craft",
    });
    goal = new google.maps.Marker({
        position: {lat: -33.722, lng:150.67398},
        map,
        title: "goal",
    });
}

sinks["/wamv/sensors/gps/gps/fix"] = (msg) => {
    latlng = {lat: msg.msg.latitude, lng: msg.msg.longitude}
    marker.setPosition(latlng);
}

sinks["/vrx/station_keeping/goal"] = (msg) => {
    latlng = {lat: msg.msg.pose.position.latitude, lng: msg.msg.pose.position.longitude}
    goal.setPosition(latlng);
}

sinks["/wamv/sensors/gps/gps/fix_velocity"] = (msg) => {
    var x = msg.msg.vector.x.toFixed(6);
    var y = msg.msg.vector.y.toFixed(6);
    document.getElementById("gps_vel_x").innerHTML = x;
    document.getElementById("gps_vel_y").innerHTML = y;
    var ctx = document.getElementById("gps_vis").getContext("2d");
    ctx.clearRect(0,0,100,100);
    ctx.beginPath();
    ctx.moveTo(50,50);
    ctx.lineTo(50+x*20, 50+y*20);
    ctx.stroke();
}

sinks["/wamv/sensors/imu/imu/data"] = (msg) => {
    var eu = new THREE.Euler();
    var ex = new THREE.Quaternion(msg.msg.orientation.x, msg.msg.orientation.y, msg.msg.orientation.z, msg.msg.orientation.w);
    eu.setFromQuaternion(ex)
    document.getElementById("imu_direction").innerHTML = (eu.z).toFixed(6);
    document.getElementById("imu_vel_z").innerHTML = msg.msg.angular_velocity.x.toFixed(6);
    var ctx = document.getElementById("imu_vis").getContext("2d");
    ctx.clearRect(0,0,100,100);
    ctx.beginPath();
    ctx.moveTo(50,50);
    ctx.lineTo(50+Math.cos(eu.z)*45, 50-Math.sin(eu.z)*45);
    ctx.stroke();
}

var dir;
var speed;
sinks["/vrx/debug/wind/direction"] = (msg) => {
    dir = msg.msg.data.toFixed(6);
    document.getElementById("wind_direction").innerHTML = dir;
}
sinks["/vrx/debug/wind/speed"] = (msg) => {
    speed = msg.msg.data.toFixed(6);
    document.getElementById("wind_speed").innerHTML = speed;
    var ctx = document.getElementById("wind_vis").getContext("2d");
    ctx.clearRect(0,0,100,100);
    ctx.beginPath();
    ctx.moveTo(50,50);
    console.log(speed);
    ctx.lineTo(50+Math.cos(dir*Math.PI/180)*speed*10, 50+Math.sin(dir*Math.PI/180)*speed*10);
    ctx.stroke();
}

sinks["/vrx/station_keeping/pose_error"] = (msg) => {
    document.getElementById("pose_error").innerHTML = msg.msg.data.toFixed(6);
}
sinks["/vrx/station_keeping/mean_pose_error"] = (msg) => {
    document.getElementById("pose_error_mean").innerHTML = msg.msg.data.toFixed(6);
}

// send thruster data every second
setInterval(() => {
    var thrust = parseFloat(document.getElementById("thruster").value);
    console.log("trying to publish " + thrust);
    master.next({op: "publish", topic: "/wamv/thrusters/left_thrust_cmd", msg: {data: thrust}})
    master.next({op: "publish", topic: "/wamv/thrusters/right_thrust_cmd", msg: {data: thrust}})
    document.getElementById("thruster_val").innerHTML = thrust
}, 1000)

// send thruster @ 10 Hz
setInterval(() => {
    var ang = parseFloat(document.getElementById("rudder").value)
    master.next({op: "publish", topic: "/wamv/thrusters/left_thrust_angle", msg: {data: ang}})
    master.next({op: "publish", topic: "/wamv/thrusters/right_thrust_angle", msg: {data: ang}})
    document.getElementById("rudder_val").innerHTML = ang
}, 100)

const { WebSocketSubject } = rxjs.webSocket;
master = new WebSocketSubject("ws://localhost:9090");
master.subscribe(
    msg => {
    sinks[msg.topic](msg);
    },
    err => console.log("ERROR: " + err),
    () => console.log("COMPLETE")
);
for(key in sinks){
    master.next({op: "subscribe", topic: key})
};

Controller.search();

window.addEventListener("gc.analog.change", (evt) => {
    console.log(evt.detail);
    if (evt.detail.name === "LEFT_ANALOG_STICK"){
        document.getElementById("thruster").value = evt.detail.position.y*-2;
    }
    if (evt.detail.name === "RIGHT_ANALOG_STICK"){
        console.log(evt.detail.position.x);
        document.getElementById("rudder").value = evt.detail.position.x*1.5;
    }
});

window.addEventListener("keydown", (evt) => {
    console.log(evt.detail);
    if (evt.key === "w"){
        document.getElementById("thruster").value = 1;
    }
    if (evt.key === "s"){
        document.getElementById("thruster").value = -1;
    }
    if (evt.key === "a"){
        document.getElementById("rudder").value = -0.7;
    }
    if (evt.key === "d"){
        document.getElementById("rudder").value = 0.7;
    }
});

window.addEventListener("keyup", (evt) => {
    console.log(evt.detail);
    if (evt.key === "w"){
        document.getElementById("thruster").value = 0;
    }
    if (evt.key === "s"){
        document.getElementById("thruster").value = 0;
    }
    if (evt.key === "a"){
        document.getElementById("rudder").value = 0;
    }
    if (evt.key === "d"){
        document.getElementById("rudderaw").value = 0;
    }
});