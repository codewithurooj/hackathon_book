import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Build Autonomous Humanoid Robots with ROS 2, AI, and NVIDIA Isaac',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://codewithurooj.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/my_book/',
  trailingSlash: false,

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'codewithurooj', // Usually your GitHub org/user name.
  projectName: 'my_book', // Usually your repo name.

  onBrokenLinks: 'warn', // Changed from 'throw' to allow build with missing pages

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  scripts: [
    '/static/scripts/text-selection.js',
  ],

  // Expose environment variables to client-side code
  clientModules: [
    require.resolve('./src/clientModules/env.js'),
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          path: '../docs', // Use existing docs directory
          sidebarPath: './sidebars.ts',
          routeBasePath: 'docs', // Serve docs at /docs
          // Remove edit links for now
          editUrl: 'https://github.com/codewithurooj/my_book/edit/main/',
        },
        blog: false, // Disable blog for textbook
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI Book',
      logo: {
        alt: 'Physical AI Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Learn',
        },
        {
          href: 'https://github.com/codewithurooj/my_book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learn',
          items: [
            {label: 'Introduction', to: '/docs/chapter-1-intro'},
            {label: 'ROS 2 Fundamentals', to: '/docs/chapter-2-ros2'},
            {label: 'Simulation', to: '/docs/chapter-3-simulation'},
            {label: 'NVIDIA Isaac', to: '/docs/chapter-4-isaac'},
            {label: 'VLA Models', to: '/docs/chapter-5-vla'},
          ],
        },
        {
          title: 'Community',
          items: [
            {label: 'GitHub', href: 'https://github.com/codewithurooj/my_book'},
            {label: 'ROS Answers', href: 'https://answers.ros.org/'},
            {label: 'NVIDIA Developer Forum', href: 'https://forums.developer.nvidia.com/'},
          ],
        },
        {
          title: 'More',
          items: [
            {label: 'Physical AI Resources', href: '/resources'},
            {label: 'Robotics Stack Exchange', href: 'https://robotics.stackexchange.com/'},
            {label: 'OpenAI API', href: 'https://platform.openai.com/docs'},
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
