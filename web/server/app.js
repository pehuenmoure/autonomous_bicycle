var express = require('express');
var app = express();
var path = require('path');
var http = require('http');
var server = http.Server(app);
var io = require('socket.io')(server);
var SerialPort = require('serialport');
var fs = require('fs');

app.set('view engine', 'ejs');

app.use(express.static(path.join(__dirname, 'public')));


//For getting PortName
if(!process.argv[2]){
	console.error("Usage: Node " + process.argv[1] + " serialport needed");
	process.exit(1);
};

//Initializing SerialPort Object
var port = new SerialPort(process.argv[2], {
	parser: SerialPort.parsers.readline('\n')
});

//Setting up array that stores incoming data
var data = new Array(1);
var row = new Array(4);

port.on('data', function(d){
	// console.log(d);
	var col = Math.floor(Math.trunc(d)/1000);
	row[col-1] = d-(col*1000);
	if(col == 4){
		data.push(row);
		console.log(row);
		io.emit('data', row);
	}
});

io.on('connect', function(socket){
	console.log('a user connected');

	socket.on('download', function(){
		console.log("Downloading data");
		new_data = dataToString(data);
		createCSVFile(new_data, "data");
	});
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




