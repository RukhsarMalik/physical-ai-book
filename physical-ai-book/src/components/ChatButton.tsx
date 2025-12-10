// physical-ai-book/src/components/ChatButton.tsx
import React from 'react';
import RobotIcon from './RobotIcon'; // NEW: selection-based question
import styles from './ChatButton.module.css';

interface ChatButtonProps {
  onClick: () => void;
}

// UPDATED: Modernize chat button
const ChatButton: React.FC<ChatButtonProps> = ({ onClick }) => {
  return (
    <button
      onClick={onClick}
      className={styles.chatButton}
      aria-label="Open chat"
    >
      <RobotIcon className={styles.chatIcon} />
    </button>
  );
};

export default ChatButton;