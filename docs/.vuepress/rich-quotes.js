// Plugin to convert GitBook rich quotes to custom containers

const types = {
	info: 'tip',
	note: 'tip',
	tag: 'tip',
	comment: 'tip',
	hint: 'tip',
	success: 'tip',
	warning: 'warning',
	caution: 'warning',
	danger: 'danger',
	quote: 'tip'
}

function replace(src) {
	return src.replace(/^> \*\*(.*?)\*\* (.*\n(>.*\n)*)/gm, function (match, type, text) {
		text = text.replace(/^>/gm, '');
		return `::: ${types[type.toLowerCase()]}\n${text}\n:::`;
	});
}

module.exports = {
	name: 'vuepress-plugin-rich-quotes',
	extendsMarkdown: (md) => {
		var _render = md.render;

		// TODO: a rough hack to replace rich quotes
		// TODO: use proper plugin api

		md.render = function(src, env) {
			src = replace(src);
			return _render.call(md, src, env);
		}
	},
	
};
