/**
 *
 * Small function used to add a marker to the map
 * @param {Number} Long Longitude of the new marker
 * @param {Number} Lat Latitude of the new marker
 * @param {String} id Id of the new marker
 */
function addMarker(Long, Lat, id){
    let baseStyle = new ol.style.Style({
        fill: new ol.style.Fill({
            color: 'rgba(45,252,173,0.85)'
        }),
        stroke: new ol.style.Stroke({
            color: 'rgba(7,158,183,0.53)',
            width:1
        }),
    })

    let marker = new ol.Feature({
        geometry: new ol.geom.Circle(
            ol.proj.fromLonLat([Long, Lat]),
            0.5
        ),
    });

    marker.setId(id)
    vectorSource.addFeature(marker)
    marker.setStyle(baseStyle)
    markerArray.push(marker)
}