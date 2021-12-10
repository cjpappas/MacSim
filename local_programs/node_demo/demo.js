const { init, startSim, stopSim } = require("./api");

// Run once after the websockets have connected to the server and subscribed to topics
const setup = () => console.log("*** Important setup ***");

// Called in a 1 second loop once the simulation is started
let count = 0;
const act = () => {
    console.log(count++, craft.getTaskInfo().state);
    craft.rotateAnticlockwise(1);
}

// We can call the init function with setup and act methods
const craft = init("macsim.duckdns.org", setup, act);

// Don't forget to start the simulation!
setTimeout(() => {
    startSim("station_keeping")
        .then(() => {
            console.log("Simualtion started!"); // Head over to the browser to checkout the simulation!
            // We can then stop the simulation later - in this case after 2 mins.
            setTimeout(() => stopSim().then(() => {
                console.log("Simulation stopped!");
                process.exit(0);
            }), 120000);
        });
}, 2000);