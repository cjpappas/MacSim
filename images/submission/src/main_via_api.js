const { init, startSim } = require("./api.js");
const {spawn} = require("child_process");


const craft = init("0.0.0.0", setup, act);

function setup(){
    console.log("setting up");
}

function act(){
    console.log("acting");
    console.log(craft.getTaskInfo())
    console.log(craft.getImages().front_left.width)
    if (craft.getTaskInfo().state === "finished"){
        process.exit();
    }
}

startSim("perception");