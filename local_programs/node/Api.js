const three = require("three");
const { webSocket } = require("rxjs/webSocket");
global.WebSocket = require("ws"); // Because StackOverflow told me too

let connection;
let data = {
    current: { velocity: {} },    
    goal: {},
    wind: {}
};

const topics = {
    "/vrx/debug/wind/direction": (msg) => {
        data.wind.direction = msg.msg.data.toFixed(6);
    },
    "/vrx/debug/wind/speed": (msg) => {
        data.wind.speed = msg.msg.data.toFixed(6);
    },
    "/vrx/station_keeping/goal": (msg) => {
        data.goal.lat = msg.msg.pose.position.latitude,
        data.goal.lng = msg.msg.pose.position.longitude
    },
    "/vrx/station_keeping/mean_pose_error": (msg) => {
        data.goal.mean_error = msg.msg.data.toFixed(6);
    },
    "/vrx/station_keeping/pose_error": (msg) => {
        data.goal.error = msg.msg.data.toFixed(6);
    },
    "/wamv/sensors/gps/gps/fix": (msg) => {
        data.current.lat = msg.msg.latitude;
        data.current.lng = msg.msg.longitude;
    },
    "/wamv/sensors/gps/gps/fix_velocity": (msg) => {
        data.current.velocity.x = msg.msg.vector.x.toFixed(6),
        data.current.velocity.y = msg.msg.vector.y.toFixed(6)
    },
    "/wamv/sensors/imu/imu/data": (msg) => {
        var eu = new three.Euler();
        var ex = new three.Quaternion(
            msg.msg.orientation.x, 
            msg.msg.orientation.y, 
            msg.msg.orientation.z, 
            msg.msg.orientation.w
        );
        eu.setFromQuaternion(ex);
        data.current.direction = (eu.z).toFixed(6);
        data.current.velocity.z = msg.msg.angular_velocity.x.toFixed(6);
    }
};

const init = (url) => {
    connection = new webSocket(url);
    // Subscribe
    connection.subscribe(
        (msg) => topics[msg.topic](msg),
        (err) => console.log("Error: " + err),
        () => console.log("Closed")
    );
    Object.keys(topics).forEach((topic) => connection.next({
        op: "subscribe",
        topic
    }));
    return {
        data,
        execute,
        moveForward,
        moveBackwards,
        rotateLeft,
        rotateRight,
        stop
    }
}

// Low-level api

const setLeftThrusterAngle = (val) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/left_thrust_angle",
        msg: { data: val }
    });

const setRightThrusterAngle = (val) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/right_thrust_angle",
        msg: { data: val }
    });

const setLeftThrusterPower = (val) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/left_thrust_cmd",
        msg: { data: val }
    });

const setRightThrusterPower = (val) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/right_thrust_cmd",
        msg: { data: val }
    });

// Turtle-based api
const moveForward = () => {
    setLeftThrusterAngle(0);
    setRightThrusterAngle(0);
    setLeftThrusterPower(1);
    setRightThrusterPower(1);
}
const moveBackwards = () => {
    setLeftThrusterAngle(0);
    setRightThrusterAngle(0);
    setLeftThrusterPower(-1);
    setRightThrusterPower(-1);
}
const rotateLeft = () => {
    setLeftThrusterAngle(-1); // outwards
    setRightThrusterAngle(1); // inwards
    setLeftThrusterPower(-1); // backwards
    setRightThrusterPower(1); // forwards
}
const rotateRight = () => {
    setLeftThrusterAngle(1); // inwards
    setRightThrusterAngle(-1); // outwards
    setLeftThrusterPower(1); // forwards
    setRightThrusterPower(-1); // backwards
}
const stop = () => {
    setLeftThrusterAngle(0);
    setRightThrusterAngle(0);
    setLeftThrusterPower(0);
    setRightThrusterPower(0);
}

const execute = (func, interval) => setInterval(func, interval);

module.exports = { init };