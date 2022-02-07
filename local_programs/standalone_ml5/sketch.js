const stream = document.getElementById("camera_stream");
const od = ml5.objectDetector("yolo", {}, () => {console.log("model loaded");});

function setup(){
	createCanvas(400, 400);
	textSize(width / 3);
	textAlign(CENTER, CENTER);

}

function draw(){
	background(200);
  od.detect(stream, (err, results) => {
    console.log(results[0]);
  });
}