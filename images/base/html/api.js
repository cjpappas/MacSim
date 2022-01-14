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

let connection;
const initialData = {
    cur_pos: { r: 0, theta: 0 },    
    cur_vel: { r: 0, theta: 0 },
    // Task 1: Station Keeping
    goal_pos: undefined,
    goal_vel: undefined,
    // Task 2: Wayfinding
    poses: undefined,
    // Task 4: Wildlife Encounter and Avoid
    animals: undefined,
    // Task 5: Channel Navigation, Acoustic Beacon Localization and Obstacle Avoidance
    blackbox_ping: undefined,
    images: {
        front_left: { height: 0, width: 0, encoding: "", step: 0, data: [] }, // https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        front_right: { height: 0, width: 0, encoding: "", step: 0, data: [] },
        middle_right: { height: 0, width: 0, encoding: "", step: 0, data: [] }
    },
    task: { name: "None", state: "Not started", ready_time: 0, running_time: 0, elapsed_time: 0, remaining_time: 0, timed_out: false, score: 0.0 },
    wind: { heading: 0, speed: 0 }
};
let data = JSON.parse(JSON.stringify(initialData)); // Deep copy
let urls = { server: "", ws: "", stream: "" };

// Capture inital craft pos - Used for conversion to polar coords
let referencePos = { lat: 0, lng: 0 }

const topics = {
    "/vrx/debug/wind/direction": {
        onMsg: (msg) => data.wind.heading = msg.data,
        msgType: "std_msgs/Float64"
    },
    "/vrx/debug/wind/speed": {
        onMsg: (msg) => data.wind.speed = msg.data,
        msgType: "std_msgs/Float64"
    },
    // "/vrx/scan_dock_deliver/color_sequence": {
    //     onMsg: (msg) => {

    //     },
    //     msgType: "vrx_gazebo/ColorSequence" // Doesn't seem to be implemented?
    // },
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
            data.goal_pos = calcPolarCoords(msg.pose.position.latitude, msg.pose.position.longitude);
            data.goal_vel = { r: 0, theta: eu.z }
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
    "/vrx/wayfinding/waypoints": {
        onMsg: (msg) => data.poses = msg.poses.map((pose) => {
            var eu = new three.Euler();
            var ex = new three.Quaternion(
                pose.pose.orientation.x, 
                pose.pose.orientation.y, 
                pose.pose.orientation.z, 
                pose.pose.orientation.w
            );
            eu.setFromQuaternion(ex);
            return {
                goal_pos: calcPolarCoords(pose.pose.position.latitude, pose.pose.position.longitude),
                goal_vel: { r: 0, theta: eu.z }
            };
        }),
        msgType: "geographic_msgs/GeoPath"
    },
    "/vrx/wildlife/animals/poses": {
        onMsg: (msg) => data.animals = msg.poses.map((pose) => {
            var eu = new three.Euler();
            var ex = new three.Quaternion(
                pose.pose.orientation.x, 
                pose.pose.orientation.y, 
                pose.pose.orientation.z, 
                pose.pose.orientation.w
            );
            eu.setFromQuaternion(ex);
            return {
                animal_pos: calcPolarCoords(pose.pose.position.latitude, pose.pose.position.longitude),
                animal_vel: { r: 0, theta: eu.z }, // This might be wrong since animals can move, but will leave as is for now
                animal_type: pose.header.frame_id
            }
        }),
        msgType: "geographic_msgs/GeoPath"
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
            if(referencePos.lat == referencePos.lng == 0){
                referencePos = { lat: msg.latitude, lng: msg.longitude };
            }
            data.cur_pos = calcPolarCoords(msg.latitude, msg.longitude);
        },
        msgType: "sensor_msgs/NavSatFix"
    },
    "/wamv/sensors/gps/gps/fix_velocity": {
        onMsg: (msg) => {
            data.cur_vel.r = Math.sqrt(Math.pow(msg.vector.x, 2) + Math.pow(msg.vector.y, 2));
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
            data.cur_vel.theta = eu.z;
        },
        msgType: "sensor_msgs/Imu"
    },
    "/wamv/sensors/pingers/pinger/range_bearing": {
        onMsg: (msg) => data.blackbox_ping = {
            range: msg.range,
            bearing: msg.bearing,
            elevation: msg.elevation
        },
        msgType: "usv_msgs/RangeBearing"
    }
};

/**
 * Create + initalise connection ans subscribe to simualtion topics.
 * @param {string} url - The url of the simualtion.
 * @returns {Object} Contains functions to interact with simulation.
 */
const init = (url, setup = undefined, act = () => {}) => {
    urls = generateUrls(url);
    connection = new ROSLIB.Ros({ url: urls.ws });
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
        } else if (env == "node"){
            const listener = new ROSLIB.Topic({
                ros: connection,
                name: topic,
                messageType: topics[topic].msgType
            });
            listener.subscribe((message) => topics[topic].onMsg(message));
        }
    });
    if(setup !== undefined) setup();
    const craft = {
        getPosition,
        getVelocity,
        getGoalPosition,
        getGoalVelocity,
        getWayfindingPositions,
        getAnimalPositions,
        getBlackboxPing,
        getImages,
        getTaskInfo,
        getWindInfo,
        imm: {
            setLeftThrusterAngle,
            setRightThrusterAngle,
            setLateralThrusterAngle,
            setLeftThrusterPower,
            setRightThrusterPower,
            setLateralThrusterPower,
            sendPerceptionGuess,
            fireBallShooter
        },
        moveForward,
        moveBackwards,
        rotateAnticlockwise,
        rotateClockwise,
        stop,
        act,
        log
    }
    const actLoop = () => {
        if(getTaskInfo().state === "running"){
            var interval = setInterval(() => {
                craft.act();
                if(getTaskInfo().state === "finished" || getTaskInfo().state === "Not started") {
                    clearInterval(interval);
                    // In case we stop the sim then start it again without refreshing
                    setTimeout(() => actLoop(), 500);
                }
            }, 1000);
        } else {
            setTimeout(() => actLoop(), 500);
        }
    }
    actLoop();
    return craft;
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

const sendPerceptionGuess = (lat, lng, objString) => {
    // List of IDs from https://robonation.org/app/uploads/sites/2/2021/09/VirtualRobotX2022_Task-Descriptions_v1.0.pdf
    const identifiers = [
        "mb_marker_buoy_black",
        "mb_marker_buoy_green",
        "mb_marker_buoy_red",
        "mb_marker_buoy_white",
        "mb_round_buoy_black",
        "mb_round_buoy_orange"
    ]
    if(objString in identifiers){
        const topic = new ROSLIB.Topic({
            ros: connection,
            name: "/vrx/perception/landmark",
            msgType: "geographic_msgs/GeoPoseStamped"
        });
        topic.publish(new ROSLIB.Message({
            header: {
                stamp: Date.now(),
                frame_id: objString
            },
            pose: {
                position: {
                    latitude: lat,
                    longitude: lng,
                    altitude: 0.0
                }
            }
        }));
    } else {
        throw new Error(`Unknown object identifier: ${objString}`);
    }
}

const fireBallShooter = () => {
    const topic = new ROSLIB.Topic({
        ros: connection,
        name: "/wamv/shooters/ball_shooter/fire",
        msgType: "std_msgs/Empty"
    });
    topic.publish(new ROSLIB.Message({}));
}

// Getters

/**
 * Returns the crafts's current position.
 * @returns {
 *     r: float - craft's current distance from reference point (initial location on load),
 *     theta: float - craft's current angle, in radians from anticlockwise from east, from reference point (initial location on load)
 * }
 */
const getPosition = () => data.cur_pos;

/**
 * Returns the boat's current velocity.
 * @returns {
 *     r: float - current speed (estimated in m/s) of the craft travelling at theta heading,
 *     theta: float - current heading of the craft in radians from anticlockwise from east
 * }
 */
const getVelocity = () => data.cur_vel;

/**
 * STATION KEEPING (TASK 1) SIMULATION ONLY
 * Returns the position of the current goal.
 * Returns undefined for other simulations.
 * @returns {
 *     r: float - goal's distance from reference point (initial location on load),
 *     theta: float - goal angle in radians from anticlockwise from east
 * } 
 */
const getGoalPosition = () => data.goal_pos;

/**
 * STATION KEEPING (TASK 1) SIMULATION ONLY
 * Returns the velocity of the current goal.
 * Returns undefined for other simulations.
 * @returns {
 *     r: float - will always return 0 since the goal isn't moving,
 *     theta: float - heading of the goal in radians from anticlockwise from east
 * } 
 */
const getGoalVelocity = () => data.goal_vel;

/**
 * WAYFINDING (TASK 2) SIMULATION ONLY
 * Returns the array of positions for the wayfinding task.
 * Returns undefined for other simulations.
 * @returns [{
 *     goal_pos: {
 *         r: float - goal's distance from reference point (initial location on load),
 *         theta: float - goal angle in radians from anticlockwise from east
 *     }
 *     goal_vel: {
 *         r: float - will always return 0 since the goal isn't moving,
 *         theta: float - heading of the goal in radians from anticlockwise from east
 *     }
 * }] 
 */
const getWayfindingPositions = () => data.poses;

/**
 * WILDLIFE (TASK 4) SIMULATION ONLY
 * Returns the array of positions for the wayfinding task.
 * Returns undefined for other simulations.
 * @returns [{
 *     goal_pos: {
 *         r: float - goal's distance from reference point (initial location on load),
 *         theta: float - goal angle in radians from anticlockwise from east
 *     }
 *     goal_vel: {
 *         r: float - will always return 0 since the goal isn't moving,
 *         theta: float - heading of the goal in radians from anticlockwise from east
 *     }
 * }] 
 */
 const getAnimalPositions = () => data.animals;

/**
 * GYMKHANA (TASK 5) SIMULATION ONLY
 * Returns the current ping from the blackbox.
 * Returns undefined for other simulations.
 * @returns {
 *     range: float - Distance to ping
 *     bearing: float - relative bearing of blackbox from USV (with noise)
 *     elevation: float - relative elevation of blackbox from USV (with noise)
 * } 
 */
const getBlackboxPing = () => data.blackbox_ping;

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

const sims = [
    "station_keeping",
    "wayfinding",
    "perception",
    "wildlife",
    "gymkhana",
    "scan_dock_deliver"
];

/**
 * Sends a request to the server to start the requested simulation.
 * @param {string} type - The type of simulation to start.
 */
const startSim = (type) => {
    if(sims.includes(type)){
        return axios.post(`${urls.server}/api/start_sim`, { sim: type });
    }
}

/**
 * Sends a request to the server to stop the current running simulation.
 */
const stopSim = () => 
    axios.post(`${urls.server}/api/stop_sim`, {})
        .then(() => data = JSON.parse(JSON.stringify(initialData)));


// Utility funciton to translate our lat/lng to polar coords
/**
 * 
 * @param {float} lat 
 * @param {float} lng 
 * @returns {
 *     r: float - distance from reference point,
 *     theta: float - angle from reference point
 * }
 */
const calcPolarCoords = (lat, lng) => ({
        r: Math.sqrt(Math.pow(lat - referencePos.lat, 2) + Math.pow(lng - referencePos.lng, 2)),
        theta: Math.atan((lat - referencePos.lat) / (lng - referencePos.lng))
});

// Utilities for determining URLs
const generateUrls = (url) => {
    return {
        server: `http://${url}`,
        ws: `ws://${url}:9090`,
        stream: `http://${url}:8080`
    }
}

// Log function to display information on the hud as well as console.log
const log = (...args) => {
    if(env === "broswer"){
        // TY stackoverflow https://stackoverflow.com/questions/20256760/javascript-console-log-to-html
        var n, i, output = "";
        for(i = 0; i < args.length; i++){
            n = args[i];
            output += "object" == typeof n && "object" == typeof JSON && "function" == typeof JSON.stringify ? output 
            += JSON.stringify(n): output += n, output;
        }
        document.getElementById("console-log-body").innerHTML = output;
    }
    console.log(...args);
}

if(env === "node"){
  module.exports = { init, startSim, stopSim, getUrls: () => urls };
}
