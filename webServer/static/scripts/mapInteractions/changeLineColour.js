function changeLineColour( id, newColour){
    let newStyle = ol.style.Style({
        stroke: new ol.style.Stroke({
            strokeColour: '#000',
            width: 5
        })
    })
    for(let i = 0; i < lineArray.length; i++){
        if(lineArray[i].getId() === id){
            lineArray[i].setStyle(newStyle)
            break
        }
    }
}