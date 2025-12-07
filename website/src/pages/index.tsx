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
    <header className={clsx('hero', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroText}>
            <span className={styles.badge}>🤖 Interactive AI-Powered Textbook</span>
            <Heading as="h1" className={styles.heroTitle}>
              {siteConfig.title}
            </Heading>
            <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              Master the fundamentals of Physical AI, Humanoid Robotics, and cutting-edge technologies with ROS 2, NVIDIA Isaac, and Vision-Language-Action Models.
            </p>
            <div className={styles.buttons}>
              <Link
                className={clsx('button button--lg', styles.primaryButton)}
                to="/docs/chapter-1-intro">
                📚 Start Learning
              </Link>
              <Link
                className={clsx('button button--lg', styles.secondaryButton)}
                to="/docs/chapter-1-intro/what-is-physical-ai">
                ⚡ Quick Start →
              </Link>
            </div>
            <div className={styles.stats}>
              <div className={styles.stat}>
                <div className={styles.statNumber}>5</div>
                <div className={styles.statLabel}>Chapters</div>
              </div>
              <div className={styles.stat}>
                <div className={styles.statNumber}>AI</div>
                <div className={styles.statLabel}>Powered</div>
              </div>
              <div className={styles.stat}>
                <div className={styles.statNumber}>100%</div>
                <div className={styles.statLabel}>Free</div>
              </div>
            </div>
          </div>
          <div className={styles.heroImage}>
            <div className={styles.imageGlow}></div>
            <div className={styles.robotIcon}>🤖</div>
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
        <div className={styles.sectionHeader}>
          <Heading as="h2">What You'll Learn</Heading>
          <p>Comprehensive coverage of Physical AI and Humanoid Robotics</p>
        </div>
        <div className={styles.bookGrid}>
          <div className={styles.bookCard}>
            <div className={styles.cardIcon}>🧠</div>
            <h3>Physical Intelligence</h3>
            <p>Understanding how machines perceive, reason about, and interact with the physical world through sensors, actuators, and intelligent control systems.</p>
            <Link to="/docs/chapter-1-intro" className={styles.cardLink}>
              Explore →
            </Link>
          </div>
          <div className={styles.bookCard}>
            <div className={styles.cardIcon}>🦾</div>
            <h3>Humanoid Robotics</h3>
            <p>Master the complex engineering challenges of designing, building, and controlling robots that mimic human form and behavior.</p>
            <Link to="/docs/chapter-2-ros2" className={styles.cardLink}>
              Explore →
            </Link>
          </div>
          <div className={styles.bookCard}>
            <div className={styles.cardIcon}>🎯</div>
            <h3>AI Integration</h3>
            <p>Learn how machine learning, computer vision, and VLA models enable robots to understand and respond intelligently.</p>
            <Link to="/docs/chapter-5-vla" className={styles.cardLink}>
              Explore →
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

function TechStackSection() {
  return (
    <section className={styles.techStack}>
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2">Technologies Covered</Heading>
          <p>Industry-standard tools and frameworks</p>
        </div>
        <div className={styles.techGrid}>
          <div className={styles.techCard}>
            <div className={styles.techIcon}>⚙️</div>
            <h4>ROS 2</h4>
            <p>Robot Operating System</p>
          </div>
          <div className={styles.techCard}>
            <div className={styles.techIcon}>🎮</div>
            <h4>NVIDIA Isaac</h4>
            <p>Simulation & Training</p>
          </div>
          <div className={styles.techCard}>
            <div className={styles.techIcon}>👁️</div>
            <h4>Computer Vision</h4>
            <p>Perception Systems</p>
          </div>
          <div className={styles.techCard}>
            <div className={styles.techIcon}>🧩</div>
            <h4>VLA Models</h4>
            <p>Vision-Language-Action</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function AIAssistantSection() {
  return (
    <section className={styles.aiSection}>
      <div className="container">
        <div className={styles.aiContent}>
          <div className={styles.aiText}>
            <span className={styles.aiBadge}>💡 Powered by RAG</span>
            <Heading as="h2">AI Learning Assistant</Heading>
            <p>Get instant answers to your questions with our integrated RAG-powered chatbot. Select any text in the book and ask for explanations!</p>
            <ul className={styles.aiFeatures}>
              <li>✅ Context-aware answers from the book</li>
              <li>✅ Select text to ask specific questions</li>
              <li>✅ Real-time streaming responses</li>
              <li>✅ Source citations for every answer</li>
            </ul>
            <div className={styles.aiCta}>
              <span>👉 Look for the chat button at the bottom right</span>
            </div>
          </div>
          <div className={styles.aiDemo}>
            <div className={styles.chatPreview}>
              <div className={styles.chatHeader}>
                <div className={styles.chatDot}></div>
                <div className={styles.chatDot}></div>
                <div className={styles.chatDot}></div>
              </div>
              <div className={styles.chatBody}>
                <div className={styles.userMessage}>What is Physical AI?</div>
                <div className={styles.botMessage}>Physical AI refers to artificial intelligence systems that can perceive, understand, and interact with the physical world...</div>
              </div>
            </div>
          </div>
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
        <TechStackSection />
        <AIAssistantSection />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
