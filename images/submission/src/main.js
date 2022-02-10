const { init, startSim, stopSim } = require("./api");

const setup = () => console.log("Setup complete");

const act = () => setInterval(() => craft.log(craft.getPosition()), 4000);

const craft = init("localhost", setup, act);
