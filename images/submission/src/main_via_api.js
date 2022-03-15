// import * as tf from '@tensorflow/tfjs';

const { init, startSim } = require("./api.js");
const tf                 = require("@tensorflow/tfjs");


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
console.log(tf);
tf.loadGraphModel("https://storage.googleapis.com/tfjs-models/savedmodel/mobilenet_v2_1.0_224/model.json")
  .then(detector => {
    console.log("made it, about to execute model");
    detector.predict(craft.getImages().front_left.data);
    // startSim("perception");
  })
  .catch((error) => console.log(error))
  .finally(() => console.log("all done (finally)"));
