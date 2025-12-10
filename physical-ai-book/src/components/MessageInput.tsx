// physical-ai-book/src/components/MessageInput.tsx
import React, { useState } from 'react';
import styles from './MessageInput.module.css';

interface MessageInputProps {
    onSendMessage: (message: string) => Promise<void>;
    isLoading: boolean;
}

const SendIcon = () => (
    <svg width="24" height="24" viewBox="0 0 24 24" fill="currentColor">
        <path d="M2.01 21L23 12 2.01 3 2 10l15 2-15 2z" />
    </svg>
);

const MessageInput: React.FC<MessageInputProps> = ({ onSendMessage, isLoading }) => {
    const [input, setInput] = useState('');

    const handleSend = async () => {
        if (input.trim() && !isLoading) {
            await onSendMessage(input);
            setInput('');
        }
    };

    const handleKeyPress = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
        if (e.key === 'Enter' && !e.shiftKey) {
            e.preventDefault();
            handleSend();
        }
    };

    return (
        <div className={styles.messageInputContainer}>
            <textarea
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder={isLoading ? 'Thinking...' : 'Type a message...'}
                disabled={isLoading}
                className={styles.messageInput}
                rows={1}
            />
            <button
                onClick={handleSend}
                disabled={!input.trim() || isLoading}
                className={styles.sendButton}
                aria-label="Send message"
            >
                <SendIcon />
            </button>
        </div>
    );
};

export default MessageInput;
