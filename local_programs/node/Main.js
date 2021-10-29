const { init } = require("./Api");

// Returns a connection to the rosbridge server
const ros = init("ws://172.17.0.3:9090");
const stationKeeping = () => {
    const goal = ros.getGoalPosition();
    const pos = ros.getPosition();
    if(goal.err > 4){
        const r = Math.atan2(pos.lng - goal.lng, pos.lat - goal.lat);
        if(r - pos.dir < 3){
            ros.moveForward();
        } else {
            ros.rotateLeft();
        }
    } else {
        ros.stop();
    }
}
ros.execute(stationKeeping, 1000);
