/**
 *
 * Small function used to add a marker to the map
 * @param {Number} Long Longitude of the new marker
 * @param {Number} Lat Latitude of the new marker
 * @param {String} id Id of the new marker
 */
function addMarker(Long, Lat, id){
    let marker = new ol.Feature({
        geometry: new ol.geom.Circle(
            ol.proj.fromLonLat([Long, Lat]),
            3
        ),
    });

    marker.setId(id)
    marker.setStyle(baseMarkerStyle)
    vectorSource.addFeature(marker)
    markerArray.push(marker)
    markerArray.sort(function (a,b){return a.getId() - b.getId()})
}