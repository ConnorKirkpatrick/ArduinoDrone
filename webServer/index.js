const path = require("path");

const app = require("express")();
const http = require("http").createServer(app);
const io = require("socket.io")(http);
const events = require("events");

const express = require("express")
app.use("/static", express.static(path.join(__dirname, "./static/")));
const pug = require("pug")
app.set("view engine","pug")
app.set("views","./static/views")

const fs = require("fs")