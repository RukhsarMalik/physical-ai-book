// physical-ai-book/src/components/ChatWindow.tsx
import React from 'react';
import styles from './ChatWindow.module.css';

interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  children: React.ReactNode;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ isOpen, onClose, children }) => {
  if (!isOpen) return null;

  return (
    <div className={styles.chatWindow}>
      <div className={styles.chatHeader}>
        <h3 className={styles.chatTitle}>AI Teaching Assistant</h3>
        <button onClick={onClose} className={styles.closeButton}>
          &times;
        </button>
      </div>
      <div className={styles.chatBody}>
        {children}
      </div>
    </div>
  );
};

export default ChatWindow;
