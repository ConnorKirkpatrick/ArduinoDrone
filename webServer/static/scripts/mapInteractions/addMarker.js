/**
 *
 * Small function used to add a marker to the map
 * @param {Number} Long Longitude of the new marker
 * @param {Number} Lat Latitude of the new marker
 * @param {String} id Id of the new marker
 */
function addMarker(Long, Lat, id){
    let marker = new ol.Feature({
        geometry: new ol.geom.Point(
            ol.proj.fromLonLat([Long, Lat])
        ),
    });
    marker.setId(id)
    vectorSource.addFeature(marker)
    markerArray.push(marker)
}