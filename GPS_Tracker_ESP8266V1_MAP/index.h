char webpage[] PROGMEM = R"=====(
<!doctype html>
<meta charset="utf-8">
<head>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css"
  integrity="sha512-xodZBNTC5n17Xt2atTPuE1HxjVMSvLVW9ocqUKLsCC5CXdbqCmblAshOMAS6/keqq/sMZMZ19scR4PsZChSR7A=="
  crossorigin=""/>
<style type="text/css">
#map{height:1000px;}
.card{
height: 380px;
padding:5px;
width:100%;
background: #1f98bd81;
border-radius: 10px;
color: #000;
margin:0px;
font-size: 300%;
line-height: 0%;
}
.button {
  background-color: #4CAF50; /* Green */
  border: none;
  color: white;
  text-align: center;
  text-decoration: none;
  display: inline;
  font-size: 190%;
  border-radius: 4px;
  font-weight: bold;
  width: 250px;
  height: 50px;
}
.button2 {background-color: #008CBA;} /* Blue */
.button3 {background-color: #f44336;} /* Red */ 
.button4 {background-color: #e7e7e7; color: black;} /* Gray */ 
.button5 {background-color: #555555;} /* Black */
</style>
<script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"
  integrity="sha512-XQoYMqMTK8LvdxXYG3nZ448hOEQiglfqkJs1NOQV44cWnUrBc8PkAOcXy20w0vlaXaVUearIOBhiXZ5V3ynxwA=="
  crossorigin=""></script>
<title>Carte de la Balise</title>
</head>
<button  class="button button5" type="button" id="dwn-btn">Générer GPX</button>
<label for="LonLat">_________Socket___</label>
<textarea id="LonLat"  style="width: 170px; height: 17px;resize: none;font-size: 100%;font-weight: bold;" readonly>Socket</textarea>

<script type="text/javascript">

function init() {

function GPXService(name) {
const self = this;
const metadata = {
name: '',
author: {
name: ''
},
description: '',
date: new Date()
};
const xmlStart = `<?xml version="1.0"?>
<gpx
xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
xmlns="http://www.topografix.com/GPX/1/1"
xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd"
version="1.1"
creator="${metadata.author.name}" >`;
const xmlEnd = '</gpx>';
/**
* Array of points. A point can be formed by latitude, longitude, elevation and time. To add a point please use self.addPoint()
* @type {Array}
*/
let points = [];
/**
* Set the file's author
* @param author
*/
self.setAuthor = function (author) {
metadata.author.name = author;
};
/**
* Adds a point
* @param point
* {
*   lat: float number (required)
*   lon: float number (required)
*   date: Date object (optional)
*   elevation: number (optional)
* }
*/
self.addPoint = function (point) {
if (point.lat && point.lon) {
points.push(point);
}
else throw Error("Point has to have 'lat' and 'lon'");
};
/**
* Returns the generated XML string in GPX format
*/
self.toXML = function () {
let xmlOutput = xmlStart;
xmlOutput += `<metadata>
<name>${metadata.name}</name>
<author><name>${metadata.author.name}</name></author>
<time>${metadata.date.toISOString()}</time>
</metadata>`;
xmlOutput += `<trk>
<name>${metadata.name}</name>
<cmt></cmt>
<desc></desc>
<src></src>
<trkseg>
`;
for (let i = 0; i < points.length; i++) {
let point = points[i];
xmlOutput += `<trkpt lat="${point.lat}" lon="${point.lon}">
`;
if (point.elevation) {
xmlOutput += `    <ele>${point.elevation}</ele>
`;
}
if (point.date) {
xmlOutput += `    <time>${point.date.toISOString()}</time>
`;
}
xmlOutput += `</trkpt>
`;
}
// And add the end
xmlOutput += `</trkseg>
</trk>
${xmlEnd}`;
return xmlOutput;
};
}
function download(filename, text) {
    var element = document.createElement('a');
    element.style.display = 'none';    
    element.setAttribute('href', 'data:text/gpx;charset=utf-8,' + encodeURIComponent(text));
    element.setAttribute('download', filename);
    document.body.appendChild(element);
    //simulate click of the created link
    element.click();
    document.body.removeChild(element);
}

GPXService = new GPXService();
GPXService.setAuthor("GPS_Tracker");

// Start file download.
document.getElementById("dwn-btn").addEventListener("click", function(){
    var filename = "Balise.gpx";
    let output = GPXService.toXML();
    //console.log (output);    
    download(filename, [output]);
}, false);
 
var Socket = new WebSocket('ws://' + window.location.hostname + ':81/');
var lat = 48.852969;
var lon = 2.349903;
var mymap = null;
// Fonction d'initialisation de la carte
// Créer l'objet "mymap" et l'insèrer dans l'élément HTML qui a l'ID "map"
mymap = L.map('map').setView([lat, lon], 15);
L.tileLayer('https://{s}.tile.openstreetmap.fr/osmfr/{z}/{x}/{y}.png', {attribution: 'données © <a href="//osm.org/copyright">OpenStreetMap</a>/ODbL - rendu <a href="//openstreetmap.fr">OSM France</a>',
minZoom: 1,
maxZoom: 20
}).addTo(mymap);

var greenIcon = new L.Icon({
iconUrl: 'http://chart.apis.google.com/chart?chst=d_map_pin_letter&chld=%E2%80%A2|2ecc71&chf=a,s,ee00FFFF',
iconSize: [15, 21],
iconAnchor: [12, 21],
popupAnchor: [1, -34],
});
var redIcon = new L.Icon({
iconUrl: 'http://chart.apis.google.com/chart?chst=d_map_pin_letter&chld=%E2%80%A2|e85141&chf=a,s,ee00FFFF',
iconSize: [30, 60],
iconAnchor: [12, 60],
popupAnchor: [1, -34],
});

Socket.onopen = function () {
connection.send('Connect ' + new Date());  
console.log('WebSocket connected: ');
};

Socket.onerror = function (error) {
console.log('WebSocket Error ', error);
};

Socket.onclose = function () {
console.log('WebSocket connection closed');
};

Socket.onmessage = function(event){
//Reception d'1 message  
console.log('WebSocket message '+ event.data);
var LonLatCmd = event.data;
document.getElementById("LonLat").value = LonLatCmd;
var socket= LonLatCmd.split(",");
var Lon = socket[1];var Lat = socket[2];var Cmd = socket[3];
//Execution commandes reçues 
switch (Cmd) {
case 'H':
//Point de départ
var popup = L.popup()
.setLatLng([Lat, Lon])
.setContent("Depart")
.openOn(mymap);
var marker = L.marker([Lat,Lon], {icon: redIcon}).addTo(mymap);
GPXService.addPoint({
    lat: Lat,
    lon: Lon,
    date: new Date(),
});
console.log('Setting Home Position');
break;
case 'M':
//Adding a marker on the map
var marker = L.marker([Lat,Lon],{icon: greenIcon}).addTo(mymap);
GPXService.addPoint({
    lat: Lat,
    lon: Lon,
    date: new Date(),
});
break;
case 'T':
//Pour les tests
let output = GPXService.toXML();
console.log (output);
break;
default:
console.log(`Sorry, we are out of ${Cmd}.`);
}
}; //end Socket.onmessage
}; //end init

setInterval(function() {
// Call a function repetatively with 1 Second interval
getData(0);
getData(1);
getData(2);
getData(3);
getData(4);
getData(5);
getData(6);
getData(7);
getData(8);
getData(9);
getData(10);
getData(11);
getData(12);
getData(13);
}, 3000); //3000mSeconds update rate

function getData(i) {
var xhttp = new XMLHttpRequest();
xhttp.onreadystatechange = function() {
if (this.readyState == 4 && this.status == 200) {
document.getElementById("Value"+i).innerHTML =
this.responseText;
}
};
xhttp.open("GET", "read"+i, true);
xhttp.send();
};
</script>
<body onload="javascript:init()">
<div class="card">
<p><span id="Value0">0</span></h6><p>
<p><span id="Value7">0</span></h5><p>
<table border="1" style="line-height: 100%;">
<tr>
<td><span id="Value4">0</span></td>
<td><span id="Value5">0</span></td>
<td><span id="Value6">0</span></td>
</tr>
<tr>
<td><span id="Value8">0</span></td>
<td><span id="Value2">0</span></td>
<td><span id="Value3">0</span></td>
</tr>
<td><span id="Value1">0</span></td>
<td><span id="Value9"></span></td>
<td><span id="Value13"></span></td>
</tr>
<td><span id="Value10">ATTENTE</span></td>
<td><span id="Value11"></span></td>
<td><span id="Value12"></span></td>
</table>
</div>
<div id="map"></div>
</body>
</html>
)=====";
