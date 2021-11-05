const { init } = require("./Api");

// Returns a connection to the rosbridge server
const ros = init("ws://172.17.0.2:9090");
setInterval(() => {
    ros.moveForward(100)
        .then(() => ros.rotateLeft(100))
        .catch((error) => console.log("Error: " + JSON.stringify(error)));
}, 5000);
