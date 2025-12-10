// physical-ai-book/src/components/MessageList.tsx
import React, { useRef, useEffect } from 'react';
import { Message } from '../contexts/ChatContext';

interface MessageListProps {
  messages: Message[];
}

const MessageList: React.FC<MessageListProps> = ({ messages }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div style={{ flexGrow: 1, overflowY: 'auto', padding: '10px' }}>
      {messages.map((message) => (
        <div
          key={message.id}
          style={{
            marginBottom: '10px',
            display: 'flex',
            justifyContent: message.role === 'user' ? 'flex-end' : 'flex-start',
          }}
        >
          <div
            style={{
              backgroundColor: message.role === 'user' ? '#DCF8C6' : '#EAEAEA',
              borderRadius: '10px',
              padding: '8px 12px',
              maxWidth: '70%',
              wordBreak: 'break-word',
            }}
          >
            {message.content}
            {message.sources && message.sources.length > 0 && (
              <div style={{ fontSize: '0.75em', marginTop: '5px', color: '#666' }}>
                Sources: {message.sources.map((source, index) => (
                  <span key={index} style={{ marginRight: '5px' }}>[{source}]</span>
                ))}
              </div>
            )}
            {message.isStreaming && (
              <span style={{ marginLeft: '5px', opacity: 0.7 }}>...</span>
            )}
          </div>
        </div>
      ))}
      <div ref={messagesEndRef} />
    </div>
  );
};

export default MessageList;