const sidebar = require('./sidebar');

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
		},
		toggleSidebar: true,
		repo: 'CopterExpress/clover',
		docsBranch: 'master',
		docsDir: 'docs',
	},
	locales: {
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
		},
		linkify: true
	},
	plugins: [
		'@vuepress/plugin-search',
		'vuepress-plugin-copy-code2',
		require('./rich-quotes')
	]
}
