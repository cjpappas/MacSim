// Documentation - https://github.com/cjpappas/MacSim/wiki/api
// let axios
let three;
let webSocket;
if(typeof window === "undefined"){
  axios = require("axios");
  three = require("three");
  webSocket = require("rxjs/webSocket").webSocket;
  global.WebSocket = require("ws"); // Because StackOverflow told me too
} else {
  three = THREE;
  webSocket = rxjs.webSocket.webSocket;
}

const MS_TO_KNOTS = 1.94384;

let connection;
let data = {
    cur_pos: { heading: 0, lat: 0, lng: 0 },    
    gps_vel: { x: 0, y: 0 },
    goal_pos: { lat: 0, lng: 0, heading: 0 },
    task: { name: "None", state: "Not started", ready_time: 0, running_time: 0, elapsed_time: 0, remaining_time: 0, timed_out: false, score: 0.0 },
    wind: { heading: 0, speed: 0 }
};

const topics = {
    "/vrx/debug/wind/direction": (msg) => {
        data.wind.heading = msg.msg.data;
    },
    "/vrx/debug/wind/speed": (msg) => {
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
        data.gps_vel.x = msg.msg.vector.x * MS_TO_KNOTS, 
        data.gps_vel.y = msg.msg.vector.y * MS_TO_KNOTS 
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

/**
 * Create + initalise connection ans subscribe to simualtion topics.
 * @param {string} url - The url of the simualtion.
 * @returns {Object} Contains functions to interact with simulation.
 */
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
        rotateAnticlockwise,
        rotateClockwise,
        stop
    }
}

// Immediate api

/**
 * Sets the angle of the left thruster.
 * @param {float} degrees - Number of degrees to turn the craft.
 */
const setLeftThrusterAngle = (degrees) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/left_thrust_angle",
        msg: { data: (degrees * Math.PI / 180) }
    });

/**
 * Sets the angle of the right thruster.
 * @param {float} degrees - Number of degrees to turn the craft.
 */
const setRightThrusterAngle = (degrees) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/right_thrust_angle",
        msg: { data: (degrees * Math.PI / 180) }
    });

/**
 * Sets the angle of the lateral thruster.
 * @param {float} degrees - Number of degrees to turn the craft.
 */
const setLateralThrusterAngle = (degrees) =>
    connection.next({
        op: "publish",
        topic: "/wmav/thrusters/lateral_thrust_angle",
        msg: { data: (degrees * Math.PI / 180) }
    });

/**
 * Sets the power of the left thruster.
 * @param {float} strength - Thruster strength.
 */
const setLeftThrusterPower = (strength) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/left_thrust_cmd",
        msg: { data: strength }
    });

/**
 * Sets the power of the right thruster.
 * @param {float} strength - Thruster strength.
 */
const setRightThrusterPower = (strength) =>
    connection.next({ 
        op: "publish",
        topic: "/wamv/thrusters/right_thrust_cmd",
        msg: { data: strength }
    });

/**
 * Sets the power of the lateral thruster.
 * @param {float} strength - Thruster strength.
 */    
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
 *     heading: float - current heading in radians,
 *     lat: float - current latitude,
 *     lng: float - current longitude
 * }
 */
const getPosition = () => data.cur_pos;

/**
 * STATION KEEPING SIMULATION ONLY
 * Returns the position of the current goal.
 * Returns undefined for other simulations.
 * @returns {
 *     lat: float - latitude of the goal,
 *     lng: float - longitude of the goal,
 *     heading: float - heading of the goal
 * } 
 */
const getGoalPosition = () => data.goal_pos;

/**
 * Returns the esimated velocity of the craft.
 * @returns {
 *     x: float - (m/s) in a north/south direction,
 *     y: float - (m/s) in a west/east direction
 * }
 */
const getGPSVelocity = () => data.gps_vel;

/**
 * Return various pieces of informatino about the current task running in the simulation.
 * @returns {
 *     task: string - task name,
 *     state: string - current state of the task (initial, ready, running, finished),
 *     ready_time: int - simulation time in seconds when state will transition to ready,
 *     running_time: int - simulation time in seconds when state will transition to running,
 *     elapsed_time: int - seconds since the simulation started (current time - start time),
 *     remaining_time: int - seconds remaining until simulation times out,
 *     timed_out: boolean - whether the simulation has timed out,
 *     score: float - current score from the simulation
 * }
 */
const getTaskInfo = () => data.task;

/**
 * Returns information about the wind.
 * @returns {
 *     heading: float - current direction of the wind,
 *     speed: float - current speed of the wind
 * } 
 */
const getWindInfo = () => data.wind; 

// High-level api (promise-based functions)

/**
 * Moves the boat forward, setting both the left and right
 * thrusters to a provided value or 1.  
 * @param {float} strength - Strength of the thrusters.
 * @returns {Promise}
 */
const moveForward = (strength = 1) => new Promise((resolve, reject) => {
    try {
        setLeftThrusterAngle(0);
        setRightThrusterAngle(0);
        setLeftThrusterPower(strength);
        setRightThrusterPower(strength);
        setTimeout(() => resolve(), 1000); 
    } catch (error) {
        reject(error);
    }
})

/**
 * Moves the boat backwards, setting both the left and right
 * thrusters to a provided value or 1.  
 * @param {float} strength - Strength of the thrusters.
 * @returns {Promise}
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
 * Rotates the boat anticlockwise.  
 * @param {float} strength - Strength of the thrusters.
 * @returns {Promise}
 */
const rotateAnticlockwise = (strength = 1) => new Promise((resolve, reject) => {
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
 * Rotates the boat clockwise.  
 * @param {float} strength - Strength of the thrusters.
 * @returns {Promise}
 */
const rotateClockwise = (strength = 1) => new Promise((resolve, reject) => {
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
 * Stops the boat by turning off thrusters and resetting thruster angles.  
 * @returns {Promise}
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

const sims = ["station_keeping"];

/**
 * Sends a request to the server to start the requested simulation.
 * @param {string} type - The type of simulation to start.
 */
const startSim = (type) => {
    if(sims.includes(type)){
        axios.post("/api/start_sim", { sim: type })
          .catch((error) => console.log(error));
    }
}

/**
 * Sends a request to the server to stop the current running simulation.
 */
const stopSim = () => axios.post("/api/stop_sim", {});

if(typeof window === "undefined"){
  module.exports = { init, startSim, stopSim };
}