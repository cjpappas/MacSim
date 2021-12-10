const { init, startSim } = require("./api.js");
console.log(init);

const craft = init("ws://localhost:9090", setup, act);

startSim("station_keeping", craft);

function setup() {
  console.log("setting up");
}

function act(){
  // steering force = desire velocity - current velocity
  console.log(craft.getPosition());
  console.log("acting");
}