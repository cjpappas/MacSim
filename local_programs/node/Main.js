const { init } = require("./Api");

// Returns a connection to the rosbridge server
const ros = init("ws://172.17.0.3:9090");
const stationKeeping = () => {
    if(ros.data.goal.error > 4){
        const r = Math.atan2(ros.data.current.lng - ros.data.goal.lng, ros.data.current.lat - ros.data.goal.lat);
        console.log(r - ros.data.current.direction);
        if(r - ros.data.current.direction < 3){
            ros.moveForward();
        } else {
            ros.rotateLeft();
        }
    } else {
        ros.stop();
    }
}
ros.execute(stationKeeping, 1000);
