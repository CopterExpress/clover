var url = 'ws://' + location.hostname + ':9090';
var ros = new ROSLIB.Ros({ url });

ros.on('connection', function () {
	document.body.classList.add('connected');
});

ros.on('close', function () {
	document.body.classList.remove('connected');
	setTimeout(function() {
		try {
			ros.connect(url);
		} catch (e) {}
	}, 2000);
});

var exec = new ROSLIB.Service({ ros: ros, name : '/exec', serviceType : 'clover/Execute' });
var readLaunchFiles = new ROSLIB.Service({ ros: ros, name: '/roslaunch_editor/read', serviceType: 'roslaunch_editor/ReadLaunchFiles' });
var writeLaunchFiles = new ROSLIB.Service({ ros: ros, name: '/roslaunch_editor/write', serviceType: 'roslaunch_editor/WriteLaunchFiles' });

var p = {}; // parameters storage

function readParam(name, fromUrl, _default) {
	return new Promise(function(resolve, reject) {
		// read from url
		if (fromUrl && ((p[name] = new URL(window.location.href).searchParams.get(name)) !== null)) {
			resolve();
			return;
		}
		// read from ROS params
		new ROSLIB.Param({ ros: ros, name: '/roslaunch_editor/' + name }).get(function(val) {
			if (val === null) {
				if (_default === undefined) {
					alert('Cannot read required parameter ' + name);
					reject();
				} else {
					p[name] = _default;
					resolve();
				}
				return;
			}
			p[name] = val;
			resolve();
		})
	});
}
