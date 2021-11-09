const SerialPort = require("serialport");

function serialStart(portName){
    let port = new SerialPort(portName, {
        baudRate: 9600,
        // defaults for Arduino serial communication
        dataBits: 8,
        parity: 'none',
        stopBits: 1,
        flowControl: false
    });
    return port
}

module.exports = serialStart