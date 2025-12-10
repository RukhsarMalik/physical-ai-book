import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHero() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner, 'hero--gradient-background')}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          Physical AI & Humanoid Robotics Course
        </Heading>
        <p className={clsx('hero__subtitle', styles.heroSubtitle)}>
          Master the Future of Embodied Intelligence
        </p>
        <p className={styles.heroDescription}>
          Dive deep into ROS 2, advanced simulations with Gazebo and Unity, NVIDIA Isaac Platform for accelerated AI,
          and cutting-edge Vision-Language-Action (VLA) models for autonomous humanoids.
          This course empowers you to build intelligent robots that perceive, reason, and act.
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Master the future of Physical AI & Humanoid Robotics with our comprehensive course covering ROS 2, simulation, NVIDIA Isaac, and VLA models.">
      <HomepageHero />
      <main>
        {/* Additional content can be added here if needed */}
      </main>
    </Layout>
  );
}