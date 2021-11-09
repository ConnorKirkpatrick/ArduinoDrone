function serialWrite(port, data){
    port.write(data, function(err) {
        if (err) {
            return console.log('Error on write: ', err.message)
        }
        console.log('message written')
    })
}
module.exports = serialWrite