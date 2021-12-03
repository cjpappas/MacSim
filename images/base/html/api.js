// Documentation - https://github.com/cjpappas/MacSim/wiki/api
const env = typeof window === "undefined" ? "node" : "broswer"
let three;
if(env === "node"){
  axios = require("axios");
  three = require("three");
  ROSLIB = require("roslib");
} else {
  three = THREE;
}

const MS_TO_KNOTS = 1.94384;

let connection;
const initialData = {
    cur_pos: { heading: 0, lat: 0, lng: 0 },    
    gps_vel: { x: 0, y: 0 },
    goal_pos: undefined,
    images: {
        front_left: { height: 0, width: 0, encoding: "", step: 0, data: [] }, // https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        front_right: { height: 0, width: 0, encoding: "", step: 0, data: [] },
        middle_right: { height: 0, width: 0, encoding: "", step: 0, data: [] }
    },
    task: { name: "None", state: "Not started", ready_time: 0, running_time: 0, elapsed_time: 0, remaining_time: 0, timed_out: false, score: 0.0 },
    wind: { heading: 0, speed: 0 }
};
let data = JSON.parse(JSON.stringify(initialData)); // Deep copy

const topics = {
    "/vrx/debug/wind/direction": {
        onMsg: (msg) => data.wind.heading = msg.data,
        msgType: "std_msgs/Float64"
    },
    "/vrx/debug/wind/speed": {
        onMsg: (msg) => data.wind.speed = msg.data,
        msgType: "std_msgs/Float64"
    },
    "/vrx/station_keeping/goal": {
        onMsg: (msg) => {
            var eu = new three.Euler();
            var ex = new three.Quaternion(
                msg.pose.orientation.x, 
                msg.pose.orientation.y, 
                msg.pose.orientation.z, 
                msg.pose.orientation.w
            );
            eu.setFromQuaternion(ex);
            data.goal_pos = {
                lat: msg.pose.position.latitude,
                lng: msg.pose.position.longitude,
                heading: eu.z
            };
        },
        msgType: "geographic_msgs/GeoPoseStamped"   
    },
    "/vrx/task/info": {
        onMsg: (msg) => data.task = {
            name: msg.name,
            state: msg.state,
            ready_time: msg.ready_time.secs,
            running_time: msg.running_time.secs,
            elapsed_time: msg.elapsed_time.secs,
            remaining_time: msg.remaining_time.secs,
            timed_out: msg.timed_out,
            score: msg.score
        },
        msgType: "vrx_gazebo/Task"
    },
    "/wamv/sensors/cameras/front_left_camera/image_raw": {
        onMsg: (msg) => data.images.front_left = {
            height: msg.height,
            width: msg.width,
            encoding: msg.encoding,
            step: msg.step,
            data: msg.data
        },
        msgType: "sensor_msgs/Image"
    },
    "/wamv/sensors/cameras/front_right_camera/image_raw": {
        onMsg: (msg) => data.images.front_right = {
            height: msg.height,
            width: msg.width,
            encoding: msg.encoding,
            step: msg.step,
            data: msg.data
        },
        msgType: "sensor_msgs/Image"
    },
    "/wamv/sensors/cameras/middle_right_camera/image_raw": {
        onMsg: (msg) => data.images.middle_right = {
            height: msg.height,
            width: msg.width,
            encoding: msg.encoding,
            step: msg.step,
            data: msg.data
        },
        msgType: "sensor_msgs/Image"
    },
    "/wamv/sensors/gps/gps/fix": {
        onMsg: (msg) => {
            data.cur_pos.lat = msg.latitude;
            data.cur_pos.lng = msg.longitude;
        },
        msgType: "sensor_msgs/NavSatFix"
    },
    "/wamv/sensors/gps/gps/fix_velocity": {
        onMsg: (msg) => {
            data.gps_vel.x = msg.vector.x * MS_TO_KNOTS; 
            data.gps_vel.y = msg.vector.y * MS_TO_KNOTS;
        },
        msgType: "geometry_msgs/Vector3Stamped" 
    },
    "/wamv/sensors/imu/imu/data": {
        onMsg: (msg) => {
            var eu = new three.Euler();
            var ex = new three.Quaternion(
                msg.orientation.x, 
                msg.orientation.y, 
                msg.orientation.z, 
                msg.orientation.w
            );
            eu.setFromQuaternion(ex);
            data.cur_pos.heading = eu.z;
            },
        msgType: "sensor_msgs/Imu"
    }
};

/**
 * Create + initalise connection ans subscribe to simualtion topics.
 * @param {string} url - The url of the simualtion.
 * @returns {Object} Contains functions to interact with simulation.
 */
const init = (url, setup = undefined, act = undefined) => {
    connection = new ROSLIB.Ros({ url })
    connection.on("connection", () => console.log("Connected to rosbridge server!"));
    connection.on("error", () => setTimeout(() => init(url, setup, act), 1000));
    connection.on("close", () => console.log("Connection to rosbridge server closed."))
    Object.keys(topics).forEach((topic) => {
        if(env === "broswer" && !topic.includes("camera")){
            const listener = new ROSLIB.Topic({
                ros: connection,
                name: topic,
                messageType: topics[topic].msgType
            });
            listener.subscribe((message) => topics[topic].onMsg(message));
        }
    });
    if(setup !== undefined) setup();
    return {
        getPosition,
        getGoalPosition,
        getGPSVelocity,
        getImages,
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
        stop,
        act: act === undefined ? () => {} : act
    }
}

// Immediate api

/**
 * Sets the angle of the left thruster.
 * @param {float} degrees - Number of degrees to turn the craft.
 */
const setLeftThrusterAngle = (degrees) => {
    const topic = new ROSLIB.Topic({
        ros: connection,
        name: "/wamv/thrusters/left_thrust_angle",
        msgType: "std_msgs/Float32"
    });
    topic.publish(new ROSLIB.Message({
        data: degrees * Math.PI / 180
    }));
}

/**
 * Sets the angle of the right thruster.
 * @param {float} degrees - Number of degrees to turn the craft.
 */
const setRightThrusterAngle = (degrees) => {
    const topic = new ROSLIB.Topic({
        ros: connection,
        name: "/wamv/thrusters/right_thrust_angle",
        msgType: "std_msgs/Float32"
    });
    topic.publish(new ROSLIB.Message({
        data: degrees * Math.PI / 180
    }));
}

/**
 * Sets the angle of the lateral thruster.
 * @param {float} degrees - Number of degrees to turn the craft.
 */
const setLateralThrusterAngle = (degrees) => {
    const topic = new ROSLIB.Topic({
        ros: connection,
        name: "/wamv/thrusters/lateral_thrust_angle",
        msgType: "std_msgs/Float32"
    });
    topic.publish(new ROSLIB.Message({
        data: degrees * Math.PI / 180
    }));
}

/**
 * Sets the power of the left thruster.
 * @param {float} strength - Thruster strength.
 */
const setLeftThrusterPower = (strength) => {
    const topic = new ROSLIB.Topic({
        ros: connection,
        name: "/wamv/thrusters/left_thrust_cmd",
        msgType: "std_msgs/Float32"
    });
    topic.publish(new ROSLIB.Message({
        data: strength
    }));
}

/**
 * Sets the power of the right thruster.
 * @param {float} strength - Thruster strength.
 */
const setRightThrusterPower = (strength) => {
    const topic = new ROSLIB.Topic({
        ros: connection,
        name: "/wamv/thrusters/right_thrust_cmd",
        msgType: "std_msgs/Float32"
    });
    topic.publish(new ROSLIB.Message({
        data: strength
    }));
}

/**
 * Sets the power of the lateral thruster.
 * @param {float} strength - Thruster strength.
 */    
const setLateralThrusterPower = (strength) => {
    const topic = new ROSLIB.Topic({
        ros: connection,
        name: "/wamv/thrusters/lateral_thrust_cmd",
        msgType: "std_msgs/Float32"
    });
    topic.publish(new ROSLIB.Message({
        data: strength
    }));
}

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

/**
 * Returns the currenct image from each of the three cameras.
 * @returns {
 *     front_left: { height: int, width: int, encoding: string, step: int, data: [int] },
 *     front_right: { height: int, width: int, encoding: string, step: int, data: [int] },
 *     middle_right: { height: int, width: int, encoding: string, step: int, data: [int] }
 * }
 */
const getImages = () => data.images;

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
const startSim = (type, craft) => {
    if(sims.includes(type)){
        data.task.state = "initialising";
        var interval = setInterval(() => {
            craft.act();
            if(getTaskInfo().state === "finished" || getTaskInfo().state === "Not started") clearInterval(interval);
        }, 1000);
        return axios.post("/api/start_sim", { sim: type });
    }
}

/**
 * Sends a request to the server to stop the current running simulation.
 */
const stopSim = () => 
    axios.post("/api/stop_sim", {}).then(() => data = JSON.parse(JSON.stringify(initialData)));

if(env === "node"){
  module.exports = { init, startSim, stopSim };
}