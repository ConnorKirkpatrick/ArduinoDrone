/**
 * Function used to create a line between two points on the map
 * @param {ol.Feature} marker1 The Start point of the line
 * @param {ol.Feature} marker2 The End point of the line
 */
function addLine(marker1, marker2){
    let point1 = marker1.getGeometry().getFlatCoordinates()
    let point2 = marker2.getGeometry().getFlatCoordinates()

    let newStyle = new ol.style({
            stroke: new ol.stroke.Stroke(),
        })

    let line = new ol.Feature({
        geometry: new ol.geom.LineString(
            [point1, point2]
        ),
        style: newStyle
    })
    let id1 = marker1.getId()
    let id2 = marker2.getId()
    let code = id1 +"-"+id2
    line.setId(code)
    vectorSource.addFeature(line)
    console.log(line.getStyle())
    lineArray.push(line)
}