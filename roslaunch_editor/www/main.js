var editorElem = document.querySelector('form.editor');

// escape html
function esc(unsafe) {
	return unsafe
		.replace(/&/g, "&amp;")
		.replace(/</g, "&lt;")
		.replace(/>/g, "&gt;")
		.replace(/"/g, "&quot;")
		.replace(/'/g, "&#039;");
 }

function getType(value) {
	if (value == 'true' || value == 'false') {
		return 'bool';
	} else if (Number.isInteger(Number(value))) {
		return 'int';
	} else if (!isNaN(Number(value))) {
		return 'float'
	} else {
		return 'string';
	}
}

var items = {};

function generateForm(item, content) {
	var parser = new DOMParser();
	var doc = items[item].doc = parser.parseFromString(content, 'text/xml');
	var html = '';

	// go though all arg tags
	var args = doc.querySelectorAll('launch > arg');
	for (var arg of args) {
		var name = arg.getAttribute('name');
		var comment = '';
		var value = arg.getAttribute('default');
		if (value === null) value = '';
		var type = getType(value);

		// get comment from previous sibling comment with no more than one line break in-between -->
		var prev = arg.previousSibling;
		if (prev && prev.nodeType == Node.TEXT_NODE && prev.textContent.split('\n').length <= 2) {
			prev = arg.previousSibling.previousSibling;
			if (prev && prev.nodeType == Node.COMMENT_NODE && prev != next /* don't use one comment twice */) {
				prevArgPrevComment = prev;
				comment = prev.textContent;
			}
		}

		// get comment from next sibling comment without line breaks in-between (more priority)
		var next = arg.nextSibling;
		if (next.nodeType == Node.TEXT_NODE && next.textContent.indexOf('\n') == -1) {
			next = next.nextSibling;
		}
		if (next.nodeType == Node.COMMENT_NODE) {
			comment = next.textContent;
		}

		// get options
		var options = comment.match(/^(.*?): ((.*), (.*))$/);
		if (options) {
			comment = options[1];
			options = options[2].split(',');
			options = options.map(function(option) { return option.trim(); });
		}

		if (comment.indexOf('noeditor') != -1) continue;

		if (p.hide_uncommented && !comment) continue;

		var path = `launch > arg[name=${name}]`;
		html += `<label><code title="${name}">${name}</code>`;

		if (type == 'bool') {
			html += `<span class=switch><input data-item="${esc(item)}" data-path="${path}" type=checkbox ${value == 'true' ? 'checked' : ''}><i></i></span>`;
			// html += `<label>${unslugName}<input value=${value} list=bool></label>`;
		} else if (type == 'int') {
			html += `<input data-item="${esc(item)}" data-path="${path}" type=number value="${esc(value)}">`;
		} else if (type == 'float') {
			let step = '0.1';
			try { step = '0.' + '0'.repeat(value.match(/\.(.*)$/)[1].length - 1) + '1'; } catch {}; // TODO: scientific
			html += `<input data-item="${esc(item)}" data-path="${path}" type=number step="${step}" value="${esc(value)}">`;
		} else if (options) {
			if (!options.includes(value)) {
				options.unshift(value);
			}
			var optionsHTML = options.map(function(option) {
				return `<option${option == value ? ' selected' : ''}>${esc(option)}</option>`;
			}).join('');
			html += `<select data-item="${esc(item)}" data-path="${path}">${optionsHTML}</select>`;
		} else {
			html += `<input data-item="${esc(item)}" data-path="${path}" value="${esc(value)}">`;
		}

		html += `<span class=comment title="${esc(comment)}">${esc(comment)}</span></label>`
	}

	return html;
}

function updateDocs() {
	editorElem.querySelectorAll('input, select').forEach(function(elem) {
		var type = elem.getAttribute('type');
		if (type == 'checkbox') {
			var value = String(elem.checked);
		} else {
			var value = elem.value;
		}
		var path = elem.getAttribute('data-path');
		var item = elem.getAttribute('data-item');
		var element = items[item].doc.querySelector(path);
		if (element.getAttribute('default') === null && value == '') {
			// don't add empty default if it's not set
			return;
		}
		element.setAttribute('default', value);
	});
}

editorElem.addEventListener('change', updateDocs);

function addLaunchFile(name, content) {
	html = `<fieldset><legend><code>${name}</code></legend>`;
	html += generateForm(name, content);
	html += '</fieldset>'
	editorElem.innerHTML += html;
}

function parseItem(str) {
	var parsed = str.match(/(.*?)\/(.*)/);
	var package = parsed[1];
	var name = parsed[2];
	return items[str] = { package, name };
}

var clover;

function determineMode() {
	return new Promise(function(resolve, reject) {
		function errcb(err) {
			alert(err);
			reject();
		}
		// check roslaunch_editor's read service
		ros.getServiceType('/roslaunch_editor/read', function(t) {
			if (t == 'roslaunch_editor/ReadLaunchFiles') {
				clover = false;
				resolve();
				return;
			}
			// check clover's exec service
			ros.getServiceType('/exec', function(t) {
				if (t == 'clover/Execute') {
					clover = true;
					resolve();
					return;
				}
				alert('Neither /roslaunch_editor/read nor /exec service not found');
				reject();
			}, errcb);
		}, errcb);
	});
}

function errCallback(err) {
	alert('Error calling service: ' + err);
}

function loadItems() {
	editorElem.innerHTML = '';

	if (typeof p.items == 'string') {
		p.items = p.items.split(',');
	}

	if (clover) {
		const boundary = '===BOUNDARY===';
		var cmd = 'bash -ic "' + p.items.map(function(item) {
			parseItem(item);
			return `roscat ${items[item].package} ${items[item].name}`;
		}).join(` && echo -n ${boundary} && `) + '"';
		exec.callService(new ROSLIB.ServiceRequest({ cmd }), function(res) {
			if (res.code != 0) {
				alert('Error reading launch-files');
				return;
			}
			res.output.split(boundary).forEach(function(content, i) {
				addLaunchFile(p.items[i], content);
			});
			document.body.classList.add('loaded');
		}, errCallback);

	} else {
		var req = new ROSLIB.ServiceRequest();
		req.files = p.items.map(function(item) {
			parseItem(item);
			return { 'package': items[item].package, 'name': items[item].name }
		});
		readLaunchFiles.callService(req, function(res) {
			res.files.forEach(function(item, i) {
				addLaunchFile(p.items[i], item.content);
			});
			document.body.classList.add('loaded');
		}, errCallback);
	}
}

// load params
Promise.all([
	determineMode(),
	readParam('apply_command', false, ''),
	readParam('items', true),
	readParam('hide_uncommented', true, false),
	readParam('standalone', false, false),
]).then(() => loadItems());

function shellEscape(a) {
	// https://github.com/xxorax/node-shell-escape
	var ret = [];

	a.forEach(function (s) {
		if (!['&&', '||', '|', '>', '<', '>>', '<<'].includes(s) && /[^A-Za-z0-9_\/:=-]/.test(s)) {
			s = "'" + s.replace(/'/g, "'\\''") + "'";
			s = s.replace(/^(?:'')+/g, '') // unduplicate single-quote at the beginning
				.replace(/\\'''/g, "\\'"); // remove non-escaped single-quote if there are enclosed between 2 escaped
		}
		ret.push(s);
	});

	return ret.join(' ');
}

var applying = false;

// TODO: reread launch file on connected
function apply() {
	applying = true;
	document.body.classList.remove('loaded');

	updateDocs();

	if (clover) {
		var script = '';
		for (item in items) {
			if (script) script += ' && ';
			var content = items[item].doc.documentElement.outerHTML;
			script += `_roscmd ${items[item].package} ${items[item].name} && echo ${shellEscape([content])} > $arg`;
		}
		if (p.apply_command) {
			script + ' && ' + p.apply_command;
		}

		var cmd = shellEscape(['bash', '-ic', script]);

		exec.callService(new ROSLIB.ServiceRequest({ cmd: cmd }), function(res) {
			if (res.code != 0) {
				alert('Error');
				return;
			}
			document.body.classList.add('loaded');
		});
	} else {

		var req = new ROSLIB.ServiceRequest();
		req.files = Object.keys(items).map(function(key) {
			var item = items[key];
			return { package: item.package, name: item.name, content: item.doc.documentElement.outerHTML }
		});

		writeLaunchFiles.callService(req, function(res) {
			if (!res.success) {
				alert('Error writing files ' + res.message);
			}
			document.body.classList.add('loaded');
			applying = false;
		}, errCallback);
	}
}

ros.on('connection', function() {
	if (applying) {
		// connection after applying tells that system restarted
		document.body.classList.add('loaded');
		applying = false;
	}
})
