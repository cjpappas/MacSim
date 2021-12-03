const fs = require("fs");

// Check args
if(process.argv.length != 3){
    console.error(`Wrong number of arguments supplied. Expected 3, got ${process.argv.length}`);
    process.exit(1);
}
// Read from file
fs.readFile(process.argv[2], "utf8", (err, data) => {
    if(err){
        console.error(err);
        process.exit(1);
    }
    const buffer = Buffer(data);
    const b64Encoded = buffer.toString("base64");
    const urlEncoded = encodeURI(b64Encoded);
    console.log(urlEncoded);
    process.exit(0);
})