import React from 'react';
import clsx from 'clsx'; // For conditional class names
import styles from './MessageBubble.module.css';
import RobotIcon from './RobotIcon';

interface MessageBubbleProps {
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: string[]; // Optional sources for assistant messages
}

const MessageBubble: React.FC<MessageBubbleProps> = ({ role, content, timestamp, sources }) => {
  return (
    <div
      className={clsx(styles.messageBubble, {
        [styles.userMessage]: role === 'user',
        [styles.assistantMessage]: role === 'assistant',
      })}
    >
      {role === 'assistant' && (
        <div className={styles.avatar}>
          <RobotIcon />
        </div>
      )}
      <div className={styles.messageContent}>
        <p>{content}</p>
        {sources && sources.length > 0 && (
          <div className={styles.sources}>
            <strong>Sources:</strong>
            <ul>
              {sources.map((source, index) => (
                <li key={index}>
                  <a href={`#${source}`} onClick={(e) => { e.preventDefault(); /* Handle navigation to source */ }}>
                    {source}
                  </a>
                </li>
              ))}
            </ul>
          </div>
        )}
        <span className={styles.timestamp}>{timestamp}</span>
      </div>
    </div>
  );
};

export default MessageBubble;