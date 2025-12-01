import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="text--center">
          <div className="avatar avatar--vertical margin-bottom--md">
            <img
              src="/img/physical-ai-logo.svg"
              alt="Physical AI & Humanoid Robotics Logo"
              style={{maxWidth: '250px', borderRadius: '10px'}}
            />
          </div>
          <Heading as="h1" className="hero__title">
            {siteConfig.title}
          </Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link
              className="button button--secondary button--lg"
              to="/docs/chapter-1-intro">
              Start Reading
            </Link>
            <Link
              className="button button--primary button--lg"
              to="/docs/chapter-1-intro/01-what-is-physical-ai">
              Explore Chapter 1
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeaturedBookSection() {
  return (
    <section className={styles.featuredBook}>
      <div className="container">
        <Heading as="h2">About This Book</Heading>
        <div className={styles.bookGrid}>
          <div className={styles.bookCard}>
            <h3>Physical Intelligence</h3>
            <p>Understanding the principles of how machines can perceive, reason about, and interact with the physical world through sensors, actuators, and intelligent control systems.</p>
          </div>
          <div className={styles.bookCard}>
            <h3>Humanoid Robotics</h3>
            <p>Exploring the complex engineering challenges of designing, building, and controlling robots that mimic human form and behavior for versatile interaction with human environments.</p>
          </div>
          <div className={styles.bookCard}>
            <h3>AI Integration</h3>
            <p>Learning how machine learning, computer vision, and natural language processing enable robots to understand and respond intelligently to complex real-world situations.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function ChatbotSection() {
  return (
    <section className={styles.chatbotSection}>
      <div className="container">
        <Heading as="h2">AI-Powered Learning Assistant</Heading>
        <p>Have questions about the content? Our RAG-powered chatbot can help you understand complex topics.</p>
        <div className={styles.chatContainer}>
          <p>Click the chat icon at the bottom right to start a conversation with your AI study partner!</p>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook - Interactive learning with AI assistance">
      <HomepageHeader />
      <main>
        <FeaturedBookSection />
        <HomepageFeatures />
        {/* Note: The chatbot is available as a floating button on all pages via Root.js */}
      </main>
    </Layout>
  );
}
