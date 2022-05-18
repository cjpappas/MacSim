const { init, startSim } = require("./api.js");
const tf                 = require("@tensorflow/tfjs");


const craft = init("ws://0.0.0.0:9090", setup, act);
if(craft.getTaskInfo().name == "None"){
  startSim("station_keeping", "http://0.0.0.0:80");
}

function setup(){
    console.log("setting up");
}

function act(){
    console.log("acting");
    console.log(craft.getPosition());
    craft.imm.setLeftThrusterPower(1);
    craft.imm.setRightThrusterPower(1);
    // const img_width = craft.getImages().front_left.width;
    // const img_height = craft.getImages().front_left.height;
    // const img_data = Buffer.from(craft.getImages().front_left.data, "base64").toString('binary');
    // for(var x= 0; x < 90; x++){
    //     for(var y = 0; y < 200; y++){
    //         const pos = (x*img_width + y)*3;
    //         const r = img_data.charCodeAt(pos);
    //         const g = img_data.charCodeAt(pos+1);
    //         const b = img_data.charCodeAt(pos+2);
    //         process.stdout.write(`\u001b[48;2;${r};${g};${b}m `);
    //     }
    //     process.stdout.write("\n");
    // }
    // if (craft.getTaskInfo().state === "finished"){
    //     process.exit();
}

// console.log(tf);
// tf.loadLayersModel("https://storage.googleapis.com/tfjs-models/tfjs/mobilenet_v1_0.25_224/model.json")
//   .then(detector => {
//     console.log("made it, about to execute model");
//     console.log("input shape is ", detector.inputs[0].shape);
//     console.log("output shape is ", detector.outputs[0].shape);
//     var prediction = detector.predict(tf.zeros([1,224,224,3]));
//     prediction.argMax(1).print();
//     // detector.predict(craft.getImages().front_left.data);
//     // startSim("perception");
//   })
//   .catch((error) => console.log(error))
//   .finally(() => console.log("all done with the prediction (finally)"));
