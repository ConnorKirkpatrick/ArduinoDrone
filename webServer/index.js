const path = require("path");

const app = require("express")();
const http = require("http").createServer(app);
const io = require("socket.io")(http);
const events = require("events");
const SerialPort = require("serialport")

const serialStart = require("../functions/serialStart")
const serialWrite = require("../functions/serialWrite")
const serialRead = require("../functions/serialRead")
const mapObject = require("../functions/map")

const express = require("express")
app.use("/static", express.static(path.join(__dirname, "./static/")));
const pug = require("pug")
app.set("view engine","pug")
app.set("views","./static/views")

const fs = require("fs")

let PORT = Number(process.env.PORT || 80);
http.listen(PORT, () => {
    console.log("Listening on " + PORT);
});

app.get("/", (req, res) => {
    res.sendFile(__dirname + "/static/webPages/mainPage/mainPage.html");
});

let port = serialStart('COM15')
serialRead(port)

port.on("open", () => {
    serialWrite(port, "This is a test")
})
io.on("connect", socket =>{
    console.log("Connection made")
    setTimeout(function() {
        socket.emit("newCenter", [51.24111847282226, -0.5864529744688033])
    },2000)

})
/*
/dev/ttyS6

name is '/dev/ttyS* where * is the com number -1
 */