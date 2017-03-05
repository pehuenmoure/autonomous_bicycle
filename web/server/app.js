var express = require('express');
var bodyParser = require('body-parser');
var app = express();
var path = require('path');
var http = require('http');
var server = http.Server(app);
var io = require('socket.io')(server);
var fs = require('fs');
var SerialPort = require('serialport');
var waypoints;
var data = [['lean angle','lean rate','front motor angle','rear speed','open column', 'micros']];

app.set('view engine', 'ejs');
app.use(bodyParser.json()); // for parsing application/json
app.use(express.static(path.join(__dirname, 'public')));

io.on('connect', function(socket){
	console.log('a user connected');

	socket.on('download', function(){
		console.log("Downloading data");
		new_data = dataToString(data);
		createCSVFile(new_data, "data");
	});
});

SerialPort.list( function(err, ports){
	console.log(ports)
});

var myPort = new SerialPort(process.argv[2], {
	parser: SerialPort.parsers.readline('\r\n'),
	baudRate: 115200
});

//Setting up array that stores incoming data
var data = new Array(6);
myPort.on('data', function(d){
	console.log(d);
	if(d>5000){
		data[5] = d;
		// sendToApp(JSON.stringify(data)); //Should I reinitialize the array?
		// console.log(data);
		io.emit('data-msg', data)
	}
	else{
		var col = Math.floor(Math.trunc(d)/1000);
		if (col == 1){
			data[col-1] = d-(col*1500.0);
		}else if (col == 2){
			data[col-1] = d-(2500.0);
		}else if (col == 3){
			data[col-1] = d-(3500.0);
		}else if (col == 4){
			data[col-1] = 0;//d-(4500.0);
		}
		// console.log(d)
	}
});

app.get('/', function(req, res){
	// first page
	res.render('index');
});
//app.use(express.static());

server.listen(8000, function(){
	console.log('listening on 8000');
});

//For Rendering webpages
app.get('/graph', (req, res) => {
	res.render('graph');
});
app.get('/navigation', (req, res) => {
	res.render('navigation');
});
app.get('/index', (req, res) => {
	res.render('index');
});
app.get('/about', (req, res) => {
	res.render('about');
});
app.get('/team', (req, res) => {
	res.render('team');
});
app.get('/subteams', (req, res) => {
	res.render('subteams');
});
app.get('/home', (req, res) => {
	res.render('home');
});

app.route('/getwaypoints')
	.get(function (req, res) { //Handles sending data
		res.send(JSON.stringify(waypoints));
	})
  .post(function (req, res) { //Handles recieving data
  	console.log(req.body);
  	waypoints = req.body;
  })

app.route('/datastream')
	.get(function (req, res) { //Handles sending data
		res.send(data);
	})
  .post(function (req, res) { //Handles recieving data
  	console.log(req.body);
  	data.push(req.body);
 })

//=======================FOR WRITING TO FILE======================
/*
*Converts the 2d array into a csv String
*/
function dataToString(rawdata){
	var str = "";
	//Converting data to string
	for(var i = 1;i<rawdata.length;i++){
		for(var j = 0; j<rawdata[i].length;j++){
			str+=rawdata[i][j];
			if(j<rawdata[i].length-1)
				str+=',';
		}
		str+='\n';
	}
	return str;
}

/*
*Writes data to a .csv file
*/
function createCSVFile(data, name){
	fs.writeFile(name+'.csv', data,  function(err) {
		if (err) {
			return console.error(err);
		}
	});
}




