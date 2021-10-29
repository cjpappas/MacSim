const three = require("three");
const { webSocket } = require("rxjs/webSocket");
global.WebSocket = require("ws"); // Because StackOverflow told me too

let connection;
let data = {
    current: { direction: 0, lat: 0, lng: 0, velocity: { x: 0, y: 0, z: 0 } },    
    goal: undefined,
    task: "",
    waypoints: undefined,
    wind: { direction: 0, speed: 0 }
};

const topics = {
    "/vrx/debug/wind/direction": (msg) => {
        data.wind.direction = msg.msg.data.toFixed(6);
    },
    "/vrx/debug/wind/speed": (msg) => {
        data.wind.speed = msg.msg.data.toFixed(6);
    },
    "/vrx/station_keeping/goal": (msg) => {
        data.goal.lat = msg.msg.pose.position.latitude;
        data.goal.lng = msg.msg.pose.position.longitude;
    },
    "/vrx/station_keeping/mean_pose_error": (msg) => {
        data.goal.mean_error = msg.msg.data.toFixed(6);
    },
    "/vrx/station_keeping/pose_error": (msg) => {
        data.goal.error = msg.msg.data.toFixed(6);
    },
    "/vrx/task/info": (msg) => {
        data.task = msg.msg.name;
    },
    "/vrx/wayfinding/waypoints": (msg) => {
        // FIXME
        data.waypoints = msg.msg.data;
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
        execute,
        getPosition,
        getSpeed,
        getGoalPosition,
        getTask,
        getWaypoints,
        getWindInfo,
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

// Exposed interface

/**
 * Executes the given function once every interval. 
 * @param {function} func The function to execute. 
 * @param {int} interval The interval (in milli seconds) to execute the function.
 * @returns Whatever setInterval returns? 
 */
const execute = (func, interval) => setInterval(func, interval);

// Getters - so data can't be accessed directly
/**
 * Returns the boat's current position.
 * @returns {
 *     dir: float - current heading in radians.
 *     lat: float - current latitude.
 *     lng: float - current longitude.
 * }
 */
const getPosition = () => ({
    dir: data.current.direction, 
    lat: data.current.lat,
    lng: data.current.lng
});

/**
 * Returns the boat's current velocity (speed) in each direction.
 * @returns {
 *     x: float - current x velocity (speed).
 *     y: float - current y velocity (speed).
 *     z: float - current z velocity (speed).
 * }
 */
const getSpeed = () => ({
    x: data.current.velocity.x,
    y: data.current.velocity.y,
    z: data.current.velocity.z
});

/**
 * STATION KEEPING SIMULATION ONLY
 * Returns the position of the current goal.
 * Returns undefined for other simulations.
 * @returns {
 *     err: float - current distance from goal.
 *     lat: float - latitude of the goal.
 *     lng: float - longitude of the goal.
 * } 
 */
const getGoalPosition = () => ({
    err: data.goal.error,
    lat: data.goal.lat,
    lng: data.goal.lng
});

/**
 * Returns the task the simulation is running
 * @returns {string}
 */
const getTask = () => data.task;

/**
 * WAYFINDING SIMULATION ONLY
 * Returns an array of waypoints.
 * Returns undefined for other simulations.
 * @returns dno 
 */
const getWaypoints = () => data.waypoints;

/**
 * Returns information about the wind.
 * @returns {
 *     dir: float - current direction of the wind.
 *     spd: float - current speed of the wind.
 * } 
 */
const getWindInfo = () => ({
    dir: data.wind.direction,
    spd: data.wind.speed
});

// Movement
/**
 * Moves the boat forward, setting both the left and right
 * thrusters to a provided value or 1.  
 * @param {float} val 
 */
const moveForward = (val = 1) => {
    setLeftThrusterAngle(0);
    setRightThrusterAngle(0);
    setLeftThrusterPower(val);
    setRightThrusterPower(val);
}

/**
 * Moves the boat backward, setting both the left and right
 * thrusters to a provided value or 1.  
 * @param {float} val 
 */
const moveBackwards = (val = 1) => {
    setLeftThrusterAngle(0);
    setRightThrusterAngle(0);
    setLeftThrusterPower(-val);
    setRightThrusterPower(-val);
}

/**
 * Rotates the boat left-wards (anti-clockwise).
 */
const rotateLeft = () => {
    setLeftThrusterAngle(-1); // outwards
    setRightThrusterAngle(1); // inwards
    setLeftThrusterPower(-1); // backwards
    setRightThrusterPower(1); // forwards
}

/**
 * Rotates the boat right-wards (clockwise).
 */
const rotateRight = () => {
    setLeftThrusterAngle(1); // inwards
    setRightThrusterAngle(-1); // outwards
    setLeftThrusterPower(1); // forwards
    setRightThrusterPower(-1); // backwards
}

/**
 * Stops the boat by turnning off both thrusters.
 */
const stop = () => {
    setLeftThrusterAngle(0);
    setRightThrusterAngle(0);
    setLeftThrusterPower(0);
    setRightThrusterPower(0);
}


module.exports = { init };