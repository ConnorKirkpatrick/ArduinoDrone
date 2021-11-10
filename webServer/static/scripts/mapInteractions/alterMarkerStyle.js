/**
 * Used to later the size and the colour of a point on the map
 * @param {Number} id The ID of the point to alter
 * @param {Number} size The new desired size of the point
 * @param {String} colour The new desired colour of the point
 */
function alterMarkerStyle(id, size, colour){
    //get style
    //get geometry
    let newStyle = new ol.style.Style({
        fill: new ol.style.Fill({
            color: colour
        }),
        stroke: new ol.style.Stroke({
            color: 'rgba(0,0,0,0.53)',
            width:0
        }),
    })
    for(let i = 0; i < markerArray.length; i++){
        //using equality rather than equals so that we can use strings or Numbers as ID
        if(markerArray[i].getId() == id){
            markerArray[i].getGeometry().setRadius(size)
            markerArray[i].setStyle(newStyle)
            break
        }
    }
}