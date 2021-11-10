/**
 * Used to change the colour of an existing line on the map
 * @param {String} id The ID of the line we wish to alter
 * @param {Number} width The new width of the line
 * @param {String} newColour The new colour of the targeted Line
 */
function alterLineStyle(id, width, newColour){
    let newStyle = new ol.style.Style({
        stroke: new ol.style.Stroke({
            width: width,
            color: newColour
        })
    })
    for(let i = 0; i < lineArray.length; i++){
        if(lineArray[i].getId() === id){
            lineArray[i].setStyle(newStyle)
            break
        }
    }
}