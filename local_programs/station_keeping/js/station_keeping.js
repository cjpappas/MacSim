const { init, startSim } = require("./api.js");
console.log(init);

const craft = init("localhost");

craft.moveForward(1);
let inter = setInterval(() => {
  craft.rotateAnticlockwise(10);
}, 1000);

setTimeout(() => {
  clearInterval(inter);
}, 10000);