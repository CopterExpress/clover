const fs = require('fs')

const regex = /(\s*?)\*\s\[(.*?)\]\((.*?)\)/;

exports.readSummary = function (path) {
	let sidebar = [];
	let lines = fs.readFileSync(path).toString().split('\n');
	let item = {};

	for (let line of lines) {
		if (line.startsWith('#')) continue;
		if (!line.trim()) continue;

		let match = regex.exec(line);
		if (!match) {
			console.log('cannot parse', line);
			continue;
		}
		level = match[1].length / 2;
		text = match[2];
		path = match[3].trim();

		if (level == 0) {
			if (item.path) {
				// push new item
				if (item.children) {
					sidebar.push(item);
				} else {
					sidebar.push(item.path)
				}
				item = {};
			}
			item.text = text;
			item.path = path;
			item.collapsible = true;

		} else if (level == 1) {
			if (!item.children) item.children = [];
			item.children.push(path);

		} else {
			console.log('skip', text);
		}
	}

	return sidebar;
}
