const glob = require('glob');
const sidebar = require('./sidebar');

console.log(__dirname + '/..');
let markdownFiles = glob.sync('ru/*.md', { cwd: __dirname + '/..' }).map(f => '/' + f); 

module.exports = {
	// site config
	lang: 'en-US',
	title: 'Clover',
	description: 'Clover Drone Kit',
	// theme and its config
	theme: '@vuepress/theme-default',
	themeConfig: {
		logo: 'https://vuejs.org/images/logo.png',
		// sidebar: markdownFiles
		sidebar: {
			'/ru/': sidebar.readSummary("./ru/SUMMARY.md"),
			'/en/': sidebar.readSummary("./en/SUMMARY.md"),
		},
		sidebarDepth: 0,
		locales: {
			'/en/': { selectLanguageName: 'English' },
			'/ru/': { selectLanguageName: 'Русский' },
		}
	},
	locales: {
		// The key is the path for the locale to be nested under.
		// As a special case, the default locale can use '/' as its path.
		// '/en': {
		// 	lang: 'en-US',
		// 	title: 'Clover',
		// 	description: 'Clover Drone Kit'
		// },
		'/en/': {
			lang: 'en',
			title: 'Clover',
			description: 'Clover Drone Kit'
		},
		'/ru/': {
			lang: 'ru',
			title: 'Клевер',
			description: 'Конструктор квадрокоптера «Клевер»'
		}
	},
	markdown: {
		code: {
			lineNumbers: false
		}
	},
	plugins: [
		'@vuepress/plugin-search',
		'vuepress-plugin-copy-code2'
		// ['@vuepress/plugin-search', {}]
	]
}
