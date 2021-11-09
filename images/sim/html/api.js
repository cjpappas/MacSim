let webSocket;
let three
if(typeof window === "undefined"){
  three = require("three");
  webSocket = require("rxjs/webSocket").webSocket;
  global.WebSocket = require("ws"); // Because StackOverflow told me too
} else {
  three = THREE;
  webSocket = rxjs.webSocket.webSocket;
}

/*
  - Used for station-keeping algorithm.
  - Default thruster config loaded is T: https://github.com/osrf/vrx/wiki/tutorials-PropulsionConfiguration
  - Headings stored as degrees clockwise from true north
*/

const MS_TO_KNOTS = 1.94384;

let connection;
let data = {
    cur_pos: { heading: 0, lat: 0, lng: 0 },    
    gps_vel: { x: 0, y: 0 },
    goal_pos: undefined,
    task: { name: "", state: "", ready_time: 0, running_time: 0, elapsed_time: 0, remaining_time: 0, timed_out: false, score: 0.0 },
    wind: { heading: 0, speed: 0 }
};

const topics = {
    "/vrx/debug/wind/direction": (msg) => {
        // Not sure how this comes in (assume degrees) - we want to store in degrees from true north clockwise
        data.wind.heading = msg.msg.data;
    },
    "/vrx/debug/wind/speed": (msg) => {
        // Not sure how this comes in - we want to store in knots
        data.wind.speed = msg.msg.data;
    },
    "/vrx/station_keeping/goal": (msg) => {
        var eu = new three.Euler();
        var ex = new three.Quaternion(
            msg.msg.pose.orientation.x, 
            msg.msg.pose.orientation.y, 
            msg.msg.pose.orientation.z, 
            msg.msg.pose.orientation.w
        );
        eu.setFromQuaternion(ex);
        data.goal_pos = {
            lat: msg.msg.pose.position.latitude,
            lng: msg.msg.pose.position.longitude,
            heading: eu.z
        };
    },
    "/vrx/task/info": (msg) => {
        // https://github.com/osrf/vrx/blob/master/vrx_gazebo/msg/Task.msg
        data.task = {
            name: msg.msg.name,
            state: msg.msg.state,
            ready_time: msg.msg.ready_time.secs,
            running_time: msg.msg.running_time.secs,
            elapsed_time: msg.msg.elapsed_time.secs,
            remaining_time: msg.msg.remaining_time.secs,
            timed_out: msg.msg.timed_out,
            score: msg.msg.score
        }  
    },
    "/wamv/sensors/gps/gps/fix": (msg) => {
        data.cur_pos.lat = msg.msg.latitude;
        data.cur_pos.lng = msg.msg.longitude;
    },
    "/wamv/sensors/gps/gps/fix_velocity": (msg) => {
        // Not sure how this comes in (assumption m/s) - http://wiki.ros.org/hector_gazebo_plugins - 1.3 GazeboRosGPS
        data.gps_vel.x = msg.msg.vector.x * MS_TO_KNOTS, // North
        data.gps_vel.y = msg.msg.vector.y * MS_TO_KNOTS  // West
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
        data.cur_pos.heading = eu.z;
    }
};

const init = (url) => {
    connection = new webSocket(url);
    connection.subscribe(
        (msg) => topics[msg.topic](msg),
        (err) => console.log("Subscription error: " + err),
        () => console.log("Websocket closed")
    );
    Object.keys(topics).forEach((topic) => connection.next({
        op: "subscribe",
        topic
    }));
    return {
        getPosition,
        getGoalPosition,
        getGPSVelocity,
        getTaskInfo,
        getWindInfo,
        imm: {
            setLeftThrusterAngle,
            setRightThrusterAngle,
            setLateralThrusterAngle,
            setLeftThrusterPower,
            setRightThrusterPower,
            setLateralThrusterPower
        },
        moveForward,
        moveBackwards,
        rotateLeft,
        rotateRight,
        stop
    }
}

// Low-level api
// Thruster angle value is expected to be specified in radians - max angles set in configuration file
// i.e. 90 degrees = pi/2 radians
const setLeftThrusterAngle = (degrees) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/left_thrust_angle",
        msg: { data: (degrees * Math.PI / 180) }
    });

const setRightThrusterAngle = (degrees) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/right_thrust_angle",
        msg: { data: (degrees * Math.PI / 180) }
    });

const setLateralThrusterAngle = (degrees) =>
    connection.next({
        op: "publish",
        topic: "/wmav/thrusters/lateral_thrust_angle",
        msg: { data: (degrees * Math.PI / 180) }
    });

const setLeftThrusterPower = (strength) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/left_thrust_cmd",
        msg: { data: strength }
    });

const setRightThrusterPower = (strength) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/right_thrust_cmd",
        msg: { data: strength }
    });

const setLateralThrusterPower = (strength) =>
    connection.next({
        op: "publish",
        topic: "/wamv/thrusters/lateral_thrust_cmd",
        msg: { data: strength }
    })

// Getters - so data can't be accessed directly
/**
 * Returns the boat's current position.
 * @returns {
 *     dir: float - current heading in radians.
 *     lat: float - current latitude.
 *     lng: float - current longitude.
 * }
 */
const getPosition = () => data.cur_pos;

/**
 * STATION KEEPING SIMULATION ONLY
 * Returns the position of the current goal.
 * Returns undefined for other simulations.
 * @returns {
 *     lat: float - latitude of the goal.
 *     lng: float - longitude of the goal.
 * } 
 */
const getGoalPosition = () => data.goal_pos;

const getGPSVelocity = () => data.gps_vel;

const getTaskInfo = () => data.task;

/**
 * Returns information about the wind.
 * @returns {
 *     dir: float - current direction of the wind.
 *     spd: float - current speed of the wind.
 * } 
 */
const getWindInfo = () => data.wind; 

// Movement
/**
 * Moves the boat forward, setting both the left and right
 * thrusters to a provided value or 1.  
 * @param {float} val 
 */
const moveForward = (strength = 1) => new Promise((resolve, reject) => {
    try {
        setLeftThrusterAngle(0);
        setRightThrusterAngle(0);
        setLeftThrusterPower(strength);
        setRightThrusterPower(strength);
        setTimeout(() => { 
            console.log("Moved forward!");
            resolve()
        }, 1000); 
    } catch (error) {
        reject(error);
    }
})

/**
 * Moves the boat backward, setting both the left and right
 * thrusters to a provided value or 1.  
 * @param {float} val 
 */
const moveBackwards = (strength = 1) => new Promise((resolve, reject) => {
    try {
        setLeftThrusterAngle(0);
        setRightThrusterAngle(0);
        setLeftThrusterPower(-strength);
        setRightThrusterPower(-strength);
        setTimeout(() => resolve(), 1000); 
    } catch (error) {
        reject(error);
    }
});

/**
 * Rotates the boat left-wards (anti-clockwise).
 */
const rotateLeft = (strength = 1) => new Promise((resolve, reject) => {
    try {
        setLeftThrusterAngle(-1); // outwards
        setRightThrusterAngle(1); // inwards
        setLeftThrusterPower(-strength); // backwards
        setRightThrusterPower(strength); // forwards
        setTimeout(() => resolve(), 1000); 
    } catch (error) {
        reject(error);
    }
});

/**
 * Rotates the boat right-wards (clockwise).
 */
const rotateRight = (strength = 1) => new Promise((resolve, reject) => {
    try {
        setLeftThrusterAngle(1); // inwards
        setRightThrusterAngle(-1); // outwards
        setLeftThrusterPower(strength); // forwards
        setRightThrusterPower(-strength); // backwards
        setTimeout(() => resolve(), 1000); 
    } catch (error) {
        reject(error);
    }
});

/**
 * Stops the boat by turnning off both thrusters.
 */
const stop = () => new Promise((resolve, reject) => {
    try {
        setLeftThrusterAngle(0);
        setRightThrusterAngle(0);
        setLeftThrusterPower(0);
        setRightThrusterPower(0);
        setTimeout(() => resolve(), 1000); 
    } catch (error) {
        reject(error);
    }
});

if(typeof window === "undefined"){
  module.exports = { init };
}