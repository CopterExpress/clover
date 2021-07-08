const url = 'ws://' + location.hostname + ':9090';
const ros = new ROSLIB.Ros({ url: url });

ros.on('connection', function () {
	document.body.classList.add('connected');
	init();
});

ros.on('close', function () {
	document.body.classList.remove('connected');
	setTimeout(function() {
		// reconnect
		ros.connect(url);
	}, 2000);
});

const title = document.querySelector('h1');
const topicsList = document.querySelector('#topics');
const topicMessage = document.querySelector('#topic-message');

function viewTopicsList() {
	title.innerHTML = 'Topics:';

	ros.getTopics(function(topics) {
		topicsList.innerHTML = topics.topics.map(function(topic, i) {
			const type = topics.types[i];
			if (type == 'sensor_msgs/Image') {
				let url = `${location.protocol}//${location.hostname}:8080/stream_viewer?topic=${topic}`; 
				return `<li><a href="${url}" class=topic title=${type}>${topic}</a> &#x1F5BC;</li>`;
			} else {
				return `<li><a href="?topic=${topic}" class=topic title=${type}>${topic}</a></li>`;
			}
		}).join('');
	});
}

function viewTopic(topic) {
	title.innerHTML = topic;
	topicMessage.style.display = 'block';

	ros.getTopicType(topic, function(typeStr) {
		const [pack, type] = typeStr.split('/');
		let href = `https://docs.ros.org/en/noetic/api/${pack}/html/msg/${type}.html`; // FIXME: determine distro
		title.innerHTML = `${topic} <a id="topic-type" href=${href} target="_blank">${typeStr}</a>`;
	});

	new ROSLIB.Topic({ ros: ros, name: topic }).subscribe(function(msg) {
		document.title = topic;
		if (mouseDown) return;
		topicMessage.innerHTML = yamlStringify(msg); // JSON.stringify(msg, null, 4);
	});
}

let mouseDown;

topicMessage.addEventListener('mousedown', function() { mouseDown = true; });
topicMessage.addEventListener('mouseup', function() { mouseDown = false; });

function init() {
	const params = Object.fromEntries(new URLSearchParams(window.location.search).entries());

	if (!params.topic) {
		viewTopicsList();
	} else {
		viewTopic(params.topic);
	}
}
