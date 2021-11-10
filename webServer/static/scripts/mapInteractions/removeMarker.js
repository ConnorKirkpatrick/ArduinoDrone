/**
 * Function used to remove a marker from the map
 * @param {String} id The ID of the marker to remove
 */
function removeMarker(id){
    for(let i = 0; i < markerArray.length; i++){
        if(markerArray[i].getId() === id){
            vectorSource.removeFeature(markerArray[i])
            markerArray.splice(i,1)
            break
        }
    }
}