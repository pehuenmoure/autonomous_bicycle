var express = require('express');
var app = express();
var http = require('http');
var server = http.Server(app);
var io = require('socket.io')(server);
var SerialPort = require('serialport');

app.set('view engine', 'ejs');

if(!process.argv[2]){
	console.error("Usage: Node " + process.argv[1] + " serialport needed");
	process.exit(1);
};

var port = new SerialPort(process.argv[2], {
	parser: SerialPort.parsers.readline('\r\n')
});

io.on('connect', function(socket){
	console.log('a user connected');
});

var data = Array(4);
var i = 0;
port.on('data', function(d){
	if (i == 4) {
		console.log(data);
		io.emit('data-msg', data);
		i = 0;
	} else{
		data[i] = d;
		i ++;
	}
})

app.get('/', function(req, res){
	res.render('index');
});

server.listen(8000, function() {
    console.log('listening on 8000');
});