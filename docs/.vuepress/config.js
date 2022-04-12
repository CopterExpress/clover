const sidebar = require('./sidebar');

const allowedTags = ['font', 'center', 'nobr']; // allow using some deprecated and non-standard html tags

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
			'/ru/': { 
				selectLanguageName: 'Русский',
				tip: 'СОВЕТ',
				warning: 'ВНИМАНИЕ',
				danger: 'ОПАСНО',
				toggleDarkMode: 'Переключить темную тему'
			},
		},
		toggleSidebar: true,
		repo: 'CopterExpress/clover',
		docsBranch: 'master',
		docsDir: 'docs',
		lastUpdated: false,
		contributors: false
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
		linkify: true,
	},
	extendsMarkdown(md) {
		md.use(require('markdown-it-attrs')); // to use custom headers anchors
	},
	bundlerConfig: {
		vuePluginOptions: {
			template: {
				compilerOptions: {
					isCustomElement: tag => allowedTags.includes(tag)
				}
			}
		}
	},
	plugins: [
		'@vuepress/plugin-search',
		'vuepress-plugin-copy-code2',
		require('./rich-quotes')
	]
}
