let baseLineStyle = new ol.style.Style({
    stroke: new ol.style.Stroke({
        width: 3,
    })
})

let baseMarkerStyle = new ol.style.Style({
    fill: new ol.style.Fill({
        color: 'rgba(45,93,252,0.85)'
    }),
    stroke: new ol.style.Stroke({
        color: 'rgba(0,7,140,0.53)',
        width:10
    }),
})

let baseHomeStyle = new ol.style.Style({
    fill: new ol.style.Fill({
        color: '#3b245d'
    }),
    stroke: new ol.style.Stroke({
        color: 'rgba(0,0,0,0.53)',
        width:0
    }),
})
let baseMarkerSize = 10
let baseHomeSize = 15
let currentMarkerSize = 20

let currentMarkerColour = '#ff0000'
let oldMarkerColour = '#0235b4'
let newMarkerColour = '#2eff00'

let currentLineColour = '#9900ff'
let baseLineColour ='#000000'
let baseLineWidth = 3
let currentLineWidth = 7
