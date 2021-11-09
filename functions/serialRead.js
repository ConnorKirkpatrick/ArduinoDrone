function serialListen(port){
    port.on("open", function () {
        console.log('open serial communication');
        // Listens to incoming data
        port.on('data', function () {
            console.log('Data:', port.read())
        })
    });
    port.on('error', function(err) {
        console.log('Error: ', err.message)
    })
}


module.exports = serialListen
