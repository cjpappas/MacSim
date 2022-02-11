const { init } = require("./api");

const setup = () => console.log("Setup complete");

const act = () => setInterval(() => craft.log(craft.getTaskInfo()), 4000);

const craft = init(process.env.IP, setup, act);
