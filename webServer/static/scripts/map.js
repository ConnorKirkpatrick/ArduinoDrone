/**
 * This file handles the setup of everything to do with the map, such as the map itself, the layers, view and marker array
 *
 */
let baseMapLayer = new ol.layer.Tile({
    source: new ol.source.OSM()

});

let vectorSource = new ol.source.Vector()

let markerLayer = new ol.layer.Vector({
    source: vectorSource
})

let mainView = new ol.View({
    center: ol.proj.fromLonLat([0,0]),
    zoom: 20
})

//Construct the Map Object
let map = new ol.Map({
    target: 'MapDiv',
    layers: [ baseMapLayer],
    view: mainView
});

// add style to Vector layer style map
map.addLayer(markerLayer);
let markerArray = []
let lineArray = []


