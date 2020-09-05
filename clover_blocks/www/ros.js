var url = 'ws://' + location.hostname + ':9090';
export var ros = new ROSLIB.Ros({ url });

ros.on('connection', function () {
	console.log('connection')
	document.body.classList.add('connected');
	document.title = 'connected';
});

ros.on('close', function () {
	console.log('close')
	document.body.classList.remove('connected');
	setTimeout(function() {
		ros.connect(url);
	}, 2000);
});

ros.on('error', function(err) {
});

export var params = {}; // parameters storage

export function readParam(name, fromUrl, _default) {
	return new Promise(function(resolve, reject) {
		if (params[name] !== undefined) {
			resolve();
			return;
		}
		// read from url
		if (fromUrl && ((params[name] = new URL(window.location.href).searchParams.get(name)) !== null)) {
			resolve();
			return;
		}
		// read from ROS params
		new ROSLIB.Param({ ros: ros, name: '/clover_blocks/' + name }).get(function(val) {
			if (val === null) {
				if (_default === undefined) {
					alert('Cannot read required parameter ' + name);
					reject();
				} else {
					params[name] = _default;
					resolve();
				}
				return;
			}
			params[name] = val;
			resolve();
		})
	});
}

export var runService = new ROSLIB.Service({ ros: ros, name: '/clover_blocks/run', serviceType: 'clover_blocks/Run' });
export var stopService = new ROSLIB.Service({ ros: ros, name: '/clover_blocks/stop', serviceType: 'std_srvs/Trigger' });
export var landService = new ROSLIB.Service({ ros: ros, name : '/land', serviceType : 'std_srvs/Trigger' });
