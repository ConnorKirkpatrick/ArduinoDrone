function checkData(coordinates){
    let splitCoords = coordinates.split(",")
    if(splitCoords.length !== 2){
        return false
    }
    let Lat = parseFloat(splitCoords[0])
    let Long = parseFloat(splitCoords[1])
    if(isNaN(Lat) || isNaN(Long)){
        return false
    }
    return [Lat,Long]
}